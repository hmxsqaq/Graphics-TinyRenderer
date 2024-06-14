#include "renderer.h"
#include "tga-handler.h"
#include "shader.h"

const int width  = 800;
const int height = 800;

int main(int argc, char** argv) {
    std::string model_path = R"(D:\MY\CppProjects\CLion\Graphics-TinyRenderer\model\african_head\african_head.obj)";
    std::string output_filename = "../image/output.tga";

    Camera camera ({0.0, 0.0, 10.0}, 45.0, 1.0, 0.1, 50.0);
    Model model = argc == 2 ? Model(argv[1]) : Model(model_path);
    Object object(model, {0.0, 0.0, 0.0}, 140.0, 2.5);

    Light l1 {{20, 20, 20}, {500, 500, 500}};
    Light l2 {{-20, 20, 0}, {500, 500, 500}};
    Shader shader(&model, std::vector<Light>({l1, l2}));

    Renderer renderer(width, height, Renderer::RGB);
    renderer.set_model_mat(object.angle, object.scale, object.position);
    renderer.set_projection_mat(camera.fov, camera.aspect_ratio, camera.zNear, camera.zFar);
    renderer.set_view_mat(camera.position);
    renderer.draw_triangle_list(object.triangle_list, shader);

    TGAHandler::write_tga_file(output_filename, renderer);
    return 0;
}