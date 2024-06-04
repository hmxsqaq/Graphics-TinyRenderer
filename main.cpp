#include "tgaimage.h"
#include "model.h"
#include "renderer.h"

const Color white = {255, 255, 255, 255};
const Color red =   {0,   0,   255, 255};
const Color green = {0,   255, 0,   255};
const Color blue =  {255, 0,   0,   255};

Model *model = nullptr;
const int width  = 800;
const int height = 800;
Vec3 light_dir{0.0, 0.0, -1.0};

int main(int argc, char** argv) {
    if (argc == 2) {
        model = new Model(argv[1]);
    } else {
        model = new Model("../obj/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);


    for (int i = 0; i <model->n_faces(); ++i) {
        Vec2 screen_coords[3];
        Vec3 world_coords[3];
        for (int j = 0; j < 3; ++j) {
            auto vert = model->vert(i, j);
            screen_coords[j] = {(vert.x + 1.0) * width / 2.0, (vert.y + 1.0) * height / 2.0};
            world_coords[j] = vert;
        }
        auto normal = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]).normalize();
        auto intensity = normal * light_dir;
        if (intensity > 0) {
            Color color { static_cast<uint8_t>(intensity * 255),
                          static_cast<uint8_t>(intensity * 255),
                          static_cast<uint8_t>(intensity * 255),
                          255 };
            draw_triangle_barycentric(screen_coords, image, color);
        }
    }

    image.write_tga_file("../image/output.tga");
    return 0;
}