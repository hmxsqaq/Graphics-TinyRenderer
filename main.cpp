#include "renderer.h"
#include "tga-handler.h"
#include "shader.h"

constexpr int width  = 1024;
constexpr int height = 1024;
Camera camera ({0.0, 0.0, 30.0}, 45.0, 1.0, 0.1, 1000.0);
Light light1 {{1, 1, 1},    {1, 1, 1}};
Light light2 {{-1, -1, -1}, {1, 1, 1}};
Vec3 amb_light_intensity = {5};

extern Mat<4, 4> ModelViewMatrix;
extern Mat<4, 4> ProjectionMatrix;

struct StandardVertexShader : IShader {
    explicit StandardVertexShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
        : IShader(model, std::move(lights)) {
        for (auto& light : lights_) {
            // uniform light direction in view coordinates
            light.direction = resize<3>((ModelViewMatrix * resize<4>(light.direction, 0))).normalize();
            light.intensity = resize<3>((ModelViewMatrix * resize<4>(light.intensity, 0)));
        }
    }

    void vertex(int i_face, int nth_vert, Vec4 &ret_vert) override {
        // column-major order, which is convenient for fragment shader
        varying_uv.set_col(nth_vert, model_.uv(i_face, nth_vert));
        varying_normal.set_col(nth_vert,
                               resize<3>( (ModelViewMatrix).invert_transpose() * resize<4>(model_.normal(i_face, nth_vert), 0) ));
        // model space -> world space -> view space
        ret_vert = ModelViewMatrix * resize<4>(model_.vert(i_face, nth_vert), 1);
        // store the vertex in view space for the fragment shader
        t_vert_view_space.set_col(nth_vert, resize<3>(ret_vert));
        // view space -> clip space
        ret_vert = ProjectionMatrix * ret_vert;
    }

protected:
    Mat<2, 3> varying_uv;
    Mat<3, 3> varying_normal;
    Mat<3, 3> t_vert_view_space;
};

struct GouraudShader final : StandardVertexShader {
    explicit GouraudShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
        : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        const Vec3 interpolated_normal = (varying_normal * bc).normalize();

        double lightness = 0.0;
        for (const auto&[direction, intensity] : lights_) {
            lightness += std::max(0.0, interpolated_normal * direction);
        }
        ret_color = Color{255, 255, 255, 255} * lightness;
        return true;
    }
};

struct StaticLayerValueShader final : StandardVertexShader {
    explicit StaticLayerValueShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
        : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        const Vec3 interpolated_normal = (varying_normal * bc).normalize();

        double lightness = 0.0;
        for (const auto&[direction, intensity] : lights_) {
            lightness += std::max(0.0, interpolated_normal * direction);
        }
        if (lightness > 0.85) lightness = 1;
        else if (lightness > 0.60) lightness = 0.8;
        else if (lightness > 0.45) lightness = 0.6;
        else if (lightness > 0.30) lightness = 0.45;
        else if (lightness > 0.15) lightness = 0.30;
        else lightness = 0;
        ret_color = Color{255, 255, 255, 255} * lightness;
        return true;
    }
};

struct PhongShader final : StandardVertexShader {
    explicit PhongShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
        : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        const Vec3 interpolated_normal = (varying_normal * bc).normalize();
        const Vec2 interpolated_uv = varying_uv * bc;

        double lightness = 0.0;
        for (const auto &[direction, intensity]: lights_) {
            const double diffuse = std::max(0.0, interpolated_normal * direction);
            auto [x, y, z] = (interpolated_normal * (interpolated_normal * direction) * 2 - direction).normalize();
            const double specular = std::pow(std::max(0.0, -z), model_.specular()->get_color(interpolated_uv)[0] + 5);
            lightness += diffuse + specular;
        }
        Color fragment_color = model_.diffuse()->get_color(interpolated_uv);
        for (const int i : {0, 1, 2}) {
            ret_color[i] = std::min<int>(static_cast<int>(fragment_color[i] * lightness), 255);
        }
        return true;
    }
};

struct TangentShader final : StandardVertexShader {
    explicit TangentShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
            : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        const Vec3 interpolated_normal = (varying_normal * bc).normalize();
        const Vec2 interpolated_uv = varying_uv * bc;

        const Mat<3, 3> AI = Mat<3, 3>{{
            t_vert_view_space.col(1) - t_vert_view_space.col(0),
            t_vert_view_space.col(2) - t_vert_view_space.col(0),
            interpolated_normal
        }}.invert();
        const Vec3 tangent = AI * Vec3{
            varying_uv[0][1] - varying_uv[0][0],
            varying_uv[0][2] - varying_uv[0][0],
            0};
        const Vec3 bitangent = AI * Vec3{
            varying_uv[1][1] - varying_uv[1][0],
            varying_uv[1][2] - varying_uv[1][0],
            0};
        const Mat<3, 3> TBN = Mat<3, 3>{{
            tangent.normalize(),
            bitangent.normalize(),
            interpolated_normal
        }}.transpose();
        const Vec3 mapping_normal = (TBN * model_.normal(interpolated_uv)).normalize();
        double lightness = 0.0;
        for (const auto &[direction, intensity]: lights_) {
            const double diffuse = std::max(0.0, mapping_normal * direction);
            const auto [x, y, z] = (mapping_normal * (mapping_normal * direction) * 2 - direction).normalize();
            const double specular = std::pow(std::max(-z, 0.0), model_.specular()->get_color(interpolated_uv)[0] + 5);
            lightness += diffuse + specular;
        }
        Color fragment_color = model_.diffuse()->get_color(interpolated_uv);
        for (const int i : {0, 1, 2}) {
            ret_color[i] = std::min(static_cast<int>(fragment_color[i] * lightness + amb_light_intensity[i]), 255);
        }
        return true;
    }
};

int main(int argc, char** argv) {
    std::vector<std::string> object_infos = {
            // R"(..\model\african_head\african_head.obj)",
            // R"(..\model\african_head\african_head_eye_inner.obj)",
            // R"(..\model\diablo3_pose\diablo3_pose.obj)",
            // R"(..\model\boggie\body.obj)",
            // R"(..\model\boggie\eyes.obj)",
            // R"(..\model\boggie\head.obj)",
            R"(..\model\cottage.obj)",
    };
    const std::string output_filename = "../image/output.tga";

    Renderer renderer(width, height, Color::RGB);

    for (const auto& model_path : object_infos)
    {
        Object object(model_path, {0.0, 0.0, 0.0}, 0, 1);
        set_model_mat(object.angle, object.scale, object.position);
        set_view_mat(camera.position);
        set_projection_mat(camera.fov, camera.aspect_ratio, camera.zNear, camera.zFar);
        GouraudShader shader(object.model, {light1, light2});
        renderer.draw_object(object, shader);
    }

    TGAHandler::write_tga_file(output_filename,
                               renderer.width(),
                               renderer.height(),
                               renderer.bpp(),
                               renderer.frame_data());
    return 0;
}