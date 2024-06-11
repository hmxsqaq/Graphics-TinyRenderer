#include "tgahandler.h"
#include "model.h"

const Color WHITE = {255, 255, 255, 255};
const Color RED =   {0, 0, 255, 255};
const Color GREEN = {0, 255, 0, 255};
const Color BLUE =  {255, 0, 0, 255};

Model *model = nullptr;
const int width  = 800;
const int height = 800;
Vec3 light_dir{0.0, 0.0, -1.0};

int main(int argc, char** argv) {
    if (argc == 2) {
        model = new Model(argv[1]);
    } else {
        model = new Model(R"(D:\MY\CppProjects\CLion\Graphics-TinyRenderer\model\african_head\african_head.obj)");
    }

    Renderer renderer(width, height, Renderer::RGB);

    for (int i = 0; i <model->n_faces(); ++i) {
        Triangle t;
        std::array<Vec3, 3> world_coords{};
        for (int j = 0; j < 3; ++j) {
            auto vert = model->vert(i, j);
            t[j] = {(vert.x + 1.0) * width / 2.0, (vert.y + 1.0) * height / 2.0, vert.z, 1};
            world_coords[j] = vert;
        }
        auto normal = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]).normalize();
        auto intensity = normal * light_dir;
        normal.norm();
        normal.norm2();
        if (intensity > 0) {
            Color color { static_cast<uint8_t>(intensity * 255),
                          static_cast<uint8_t>(intensity * 255),
                          static_cast<uint8_t>(intensity * 255),
                          255 };
            renderer.draw_triangle_barycentric(t, color);
        }
    }

    TGAHandler::write_tga_file("../image/output2.tga", renderer);
    return 0;
}