#include "renderer.h"
#include "model.h"
#include "tga-handler.h"

struct Camera {
    Vec3 position = {0, 0, 0};
    double fov = 0;
    double aspect_ratio = 0;
    double zNear = 0;
    double zFar = 0;
};

struct Object {
    std::vector<Triangle*> triangle_list;
    Vec3 position = {0, 0, 0};
    double angle = 0;
    double scale = 0;
};

const int width  = 800;
const int height = 800;

int main(int argc, char** argv) {
    std::string model_path = R"(D:\MY\CppProjects\CLion\Graphics-TinyRenderer\model\african_head\african_head.obj)";
    std::string output_filename = "../image/output.tga";

    Camera camera {
            .position = {0.0, 0.0, 10.0},
            .fov = 45.0,
            .aspect_ratio = 1.0,
            .zNear = 0.1,
            .zFar = 50.0
    };

    Object object {
            .position = {0.0, 0.0, 0.0},
            .angle = 140.0,
            .scale = 2.5
    };

    // load model
    Model *model = argc == 2 ? new Model(argv[1]) : new Model(model_path);
    object.triangle_list.reserve(model->n_faces());
    for (int i_face = 0; i_face < model->n_faces(); ++i_face) {
        auto* t = new Triangle();
        for (int i_vert = 0; i_vert < 3; ++i_vert) {
            t->vert[i_vert] = resize<4>(model->vert(i_face, i_vert), 1);
            t->normal[i_vert] = model->normal(i_face, i_vert);
            t->tex_coords[i_vert] = model->uv(i_face, i_vert);
        }
        object.triangle_list.emplace_back(t);
    }

    Renderer renderer(width, height, Renderer::RGB);
    renderer.set_model_mat(object.angle, object.scale, object.position);
    renderer.set_projection_mat(camera.fov, camera.aspect_ratio, camera.zNear, camera.zFar);
    renderer.set_view_mat(camera.position);
    renderer.draw_triangle_list(object.triangle_list);
    TGAHandler::write_tga_file(output_filename, renderer);
    return 0;
}