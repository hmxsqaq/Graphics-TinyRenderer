#include "renderer.h"
#include "tga-handler.h"
#include "shader.h"

const int width  = 1024;
const int height = 1024;
Camera camera ({0.0, 0.0, 10.0}, 45.0, 1.0, 0.1, 1000.0);
//constexpr Vec3       eye{1,1,3}; // camera position
//constexpr Vec3    center{0,0,0}; // camera direction
//constexpr Vec3        up{0,1,0}; // camera up vector
Light light1 {{1, 1, 1},    {1, 1, 1}};
Light light2 {{-1, -1, -1}, {1, 1, 1}};
Vec3 amb_light_intensity = {5};

extern Mat<4, 4> ModelViewMatrix;
extern Mat<4, 4> ProjectionMatrix;

struct StandardVertexShader : IShader {
public:
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
struct GouraudShader : StandardVertexShader {
    explicit GouraudShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
        : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        Vec3 interpolated_normal = (varying_normal * bc).normalize();

        double intensity = 0.0;
        for (const auto& light : lights_) {
            intensity += std::max(0.0, interpolated_normal * light.direction);
        }
        ret_color = Color{255, 255, 255, 255} * intensity;
        return true;
    }
};
struct StaticLayerValueShader : StandardVertexShader {
    explicit StaticLayerValueShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
        : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        Vec3 interpolated_normal = (varying_normal * bc).normalize();

        double intensity = 0.0;
        for (const auto& light : lights_) {
            intensity += std::max(0.0, interpolated_normal * light.direction);
        }
        if (intensity > 0.85) intensity = 1;
        else if (intensity > 0.60) intensity = 0.8;
        else if (intensity > 0.45) intensity = 0.6;
        else if (intensity > 0.30) intensity = 0.45;
        else if (intensity > 0.15) intensity = 0.30;
        else intensity = 0;
        ret_color = Color{255, 255, 255, 255} * intensity;
        return true;
    }
};
struct PhongShader : StandardVertexShader {
    explicit PhongShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
        : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        Vec3 interpolated_normal = (varying_normal * bc).normalize();
        Vec2 interpolated_uv = varying_uv * bc;

        double intensity = 0.0;
        for (const auto &light: lights_) {
            double diffuse = std::max(0.0, interpolated_normal * light.direction);
            Vec3 reflected_light = (interpolated_normal * (interpolated_normal * light.direction) * 2 - light.direction).normalize();
            double specular = std::pow(std::max(0.0, -reflected_light.z), model_.specular()->get_color(interpolated_uv)[0] + 5);
            intensity += diffuse + specular;
        }
        Color fragment_color = model_.diffuse()->get_color(interpolated_uv);
        for (int i : {0, 1, 2}) {
            ret_color[i] = std::min<int>((int)(fragment_color[i] * intensity), 255);
        }
        return true;
    }
};
struct TangentShader : StandardVertexShader {
    explicit TangentShader(const Model& model, std::vector<Light>&& lights = std::vector<Light>())
            : StandardVertexShader(model, std::move(lights)) { }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        Vec3 interpolated_normal = (varying_normal * bc).normalize();
        Vec2 interpolated_uv = varying_uv * bc;

        Mat<3, 3> AI = Mat<3, 3>{{
            t_vert_view_space.col(1) - t_vert_view_space.col(0),
            t_vert_view_space.col(2) - t_vert_view_space.col(0),
            interpolated_normal
        }}.invert();
        Vec3 tangent = AI * Vec3{
            varying_uv[0][1] - varying_uv[0][0],
            varying_uv[0][2] - varying_uv[0][0],
            0};
        Vec3 bitangent = AI * Vec3{
            varying_uv[1][1] - varying_uv[1][0],
            varying_uv[1][2] - varying_uv[1][0],
            0};
        Mat<3, 3> TBN = Mat<3, 3>{{
            tangent.normalize(),
            bitangent.normalize(),
            interpolated_normal
        }}.transpose();
        Vec3 mapping_normal = (TBN * model_.normal(interpolated_uv)).normalize();
        double intensity = 0.0;
        for (const auto &light: lights_) {
            double diffuse = std::max(0.0, mapping_normal * light.direction);
            Vec3 reflected_light = (mapping_normal * (mapping_normal * light.direction) * 2 - light.direction).normalize();
            double specular = std::pow(std::max(-reflected_light.z, 0.0), model_.specular()->get_color(interpolated_uv)[0] + 5);
            intensity += diffuse + specular;
        }
        Color fragment_color = model_.diffuse()->get_color(interpolated_uv);
        for (int i : {0, 1, 2}) {
            ret_color[i] = std::min((int)(fragment_color[i] * intensity + amb_light_intensity[i]), 255);
        }
        return true;
    }
};


int main(int argc, char** argv) {
    std::vector<std::string> model_paths = {
            R"(..\model\african_head\african_head.obj)",
            R"(..\model\african_head\african_head_eye_inner.obj)",
//            R"(..\model\diablo3_pose\diablo3_pose.obj)",
//            R"(..\model\boggie\body.obj)",
//            R"(..\model\boggie\eyes.obj)",
//            R"(..\model\boggie\head.obj)",
    };
    std::string output_filename = "../image/boggie.tga";

    Renderer renderer(width, height, Color::RGB);

    for (const auto& model_path : model_paths)
    {
        Model model = Model(model_path);
        Object object(model, {0.0, 0.0, 0.0}, 0, 3);
        set_model_mat(object.angle, object.scale, object.position);
        set_view_mat(camera.position);
        set_projection_mat(camera.fov, camera.aspect_ratio, camera.zNear, camera.zFar);
        TangentShader shader(model, {light1, light2});
        renderer.draw_object(object, shader);
    }

    TGAHandler::write_tga_file(output_filename,
                               renderer.width(),
                               renderer.height(),
                               renderer.bpp(),
                               renderer.frame_data());
    return 0;
}