#include "renderer.h"
#include "tga-handler.h"
#include "shader.h"

const int width  = 800;
const int height = 800;
Camera camera ({0.0, 0.0, 10.0}, 45.0, 1.0, 0.1, 50.0);
Light l1 {{20, 20, 20}, {500, 500, 500}};
Light l2 {{-20, 20, 0}, {500, 500, 500}};

int main(int argc, char** argv) {
    std::vector<std::string> model_paths = {
            R"(..\model\african_head\african_head.obj)",
            R"(..\model\african_head\african_head_eye_inner.obj)",
//            R"(..\model\african_head\african_head_eye_outer.obj)",
    };
    std::string output_filename = "../image/output.tga";

    Shader shader(std::vector<Light>({l1, l2}));

    Renderer renderer(width, height, Color::RGB);
    renderer.set_camera(camera);

    for (const auto& model_path : model_paths)
    {
        Model model = Model(model_path);
        Object object(model, {0.0, 0.0, 0.0}, -45.0, 2.5);
        shader.set_model(&model);
        renderer.set_object(object);
        renderer.draw_triangle_list(object.triangle_list, shader);
    }

    TGAHandler::write_tga_file(output_filename,
                               renderer.width(),
                               renderer.height(),
                               renderer.bpp(),
                               renderer.frame_data());

    return 0;
}