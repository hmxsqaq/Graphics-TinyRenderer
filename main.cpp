#include "renderer.h"
#include "tga-handler.h"
#include "shader.h"

const int width  = 1024;
const int height = 1024;
Camera camera ({0.0, 0.0, 10.0}, 45.0, 1.0, 0.1, 1000.0);
Light l1 {{1, 1, 1}, {500, 500, 500}};
Light l2 {{-20, 20, 20}, {500, 500, 500}};
Vec3 amb_light_intensity{10, 10, 10};

extern Mat<4, 4> ModelViewMatrix;
extern Mat<4, 4> ProjectionMatrix;

struct Shader : IShader {
public:
    explicit Shader(const Model& model, std::vector<Light>&& lights = std::vector<Light>()) : model_(model), lights_(std::move(lights)) {
        for (auto& light : lights_) {
            // uniform light direction in view coordinates
            light.position = resize<3>((ModelViewMatrix * resize<4>(light.position, 0))).normalize();
            light.intensity = resize<3>((ModelViewMatrix * resize<4>(light.intensity, 0))).normalize();
        }
    }

    void vertex(int i_face, int nth_vert, Vec4 &ret_vert) override {
        // column-major order, which is convenient for fragment shader
        varying_uv.set_col(nth_vert, model_.uv(i_face, nth_vert));
        varying_normal.set_col(nth_vert,
                               resize<3>( ModelViewMatrix.invert_transpose() * resize<4>(model_.normal(i_face, nth_vert), 0) ));
        // model space -> world space -> view space
        ret_vert = ModelViewMatrix * resize<4>(model_.vert(i_face, nth_vert), 1);
        // store the vertex in view space for the fragment shader
        t_vert_view_space.set_col(nth_vert, resize<3>(ret_vert));
        // view space -> clip space
        ret_vert = ProjectionMatrix * ret_vert;
    }

    bool fragment(const Vec3 &bc, Color &ret_color) override {
        Vec3 bn = (varying_normal * bc).normalize();
        for (const auto& light : lights_) {
            double intensity = std::max(0.0, bn * light.position);
            ret_color = Color{255, 255, 255, 255} * intensity;
        }
        return true;
    }

protected:
    const Model& model_;
    std::vector<Light> lights_;
    Mat<2, 3> varying_uv;
    Mat<3, 3> varying_normal;
    Mat<3, 3> t_vert_view_space;
};

int main(int argc, char** argv) {
    std::vector<std::string> model_paths = {
            R"(..\model\african_head\african_head.obj)",
            R"(..\model\african_head\african_head_eye_inner.obj)",
//            R"(..\model\african_head\african_head_eye_outer.obj)",
    };
    std::string output_filename = "../image/output.tga";

    Renderer renderer(width, height, Color::RGB);

    for (const auto& model_path : model_paths)
    {
        Model model = Model(model_path);
        Object object(model, {0.0, 0.0, 0.0}, 0, 3);
        set_model_mat(object.angle, object.scale, object.position);
        set_view_mat(camera.position);
        set_projection_mat(camera.fov, camera.aspect_ratio, camera.zNear, camera.zFar);
        Shader shader(model, {l1});
        renderer.draw_object(object, shader);
    }

    TGAHandler::write_tga_file(output_filename,
                               renderer.width(),
                               renderer.height(),
                               renderer.bpp(),
                               renderer.frame_data());
    return 0;
}