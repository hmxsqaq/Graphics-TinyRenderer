#include "tgaimage.h"
#include "model.h"
#include "renderer.h"

const Color white =  {255, 255, 255, 255};
const Color red =    {0, 0, 255, 255};
const Color green =  {0, 255, 0, 255};

Model *model = nullptr;
const int width  = 100;
const int height = 100;

int main(int argc, char** argv) {
//    if (argc == 2) {
//        model = new Model(argv[1]);
//    } else {
//        model = new Model("../obj/african_head.obj");
//    }
//
//    TGAImage image(width, height, TGAImage::RGB);
//
//    Vec3 light_dir{0.0, 0.0, -1.0};

//    for (int i = 0; i < model->n_faces(); ++i) {
//        // load face
//        std::vector<int> face = model->face(i);
//        Vec2i screen_coords[3];
//        Vec3f world_coords[3];
//        for (int j = 0; j < 3; ++j) {
//            // load vert and project to screen
//            Vec3f vert = model->vert(face[j]);
//            world_coords[j] = vert;
//            screen_coords[j] = Vec2i((int)((vert.x + 1.0f) * width / 2.0f), (int)((vert.y + 1.0f) * height / 2.0f));
//        }
//        // calculate normal of face
//        Vec3f normal = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
//        normal.normalize();
//        std::cout << normal;
//
//        float intensity = normal * light_dir;
//        if (intensity > 0)
//            triangle_barycentric(screen_coords, image, {
//                static_cast<uint8_t>(intensity * 255),
//                static_cast<uint8_t>(intensity * 255),
//                static_cast<uint8_t>(intensity * 255),
//                255});
//    }

//    image.write_tga_file("../image/output.tga");
//    image.write_tga_file("../image/african_head_simple_light_shading.tga");

    Vec2 test[3];
    test[0] = {0, 1};
    test[1] = {0, -1};
    test[2] = {2, 0};
    Vec2 p{1, 0};
    auto result = get_barycentric(test, p);
    std::cout << result;
    return 0;
}