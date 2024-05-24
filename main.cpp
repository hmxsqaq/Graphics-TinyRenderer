#include "tgaimage.h"
#include "model.h"

const TGAColor white =  TGAColor(255,   255,    255,    255);
const TGAColor red =    TGAColor(255,   0,      0,      255);
const TGAColor green =  TGAColor(0,     255,    0,      255);

Model *model = nullptr;
const int width  = 800;
const int height = 800;

// Bresenham's line algorithm
void line(Vec2i p0, Vec2i p1, TGAImage &image, const TGAColor& color) {
    // if (dx < dy) the line is steep
    // we need to sample it along y axis
    // but we can also transpose it, then we can still use x axis
    // just remember to re-transpose it when drawing it
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y)) {
        // transpose
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }

    // make it left−to−right
    if (p0.x > p1.x) {
        std::swap(p0, p1);
    }

    // use error to approximate the distance between the line and the pixel, which can improve performance
    int dx = p1.x - p0.x;
    int dy = p1.y - p0.y;
    int per_error = std::abs(dy) * 2;
    int error = 0;
    int y = p0.y;

    for (int x = p0.x; x <= p1.x; x++) {
        if (steep) {
            image.set(y, x, color); // if transposed, de−transpose
        } else {
            image.set(x, y, color);
        }
        error += per_error;
        if (error > dx) {
            y += (p1.y > p0.y ? 1 : -1);
            error -= 2 * dx;
        }
    }
}

// line sweeping triangle drawing
void triangle_line_sweeping(Vec2i p0, Vec2i p1, Vec2i p2, TGAImage &image, const TGAColor& color) {
    if (p0.y == p1.y && p1.y == p2.y) return;
    // make p0.y < p1.y < p2.y
    if (p0.y > p1.y) std::swap(p0, p1);
    if (p0.y > p2.y) std::swap(p0, p2);
    if (p1.y > p2.y) std::swap(p1, p2);

    int total_height = p2.y - p0.y;
    for (int y = 0; y <= total_height; ++y) {
        bool second_half = y > p1.y - p0.y || p0.y == p1.y;
        int segment_height = second_half ? p2.y - p1.y : p1.y - p0.y;
        float alpha = (float)y / (float)total_height;
        float beta = (float)(y - (second_half ? p1.y - p0.y : 0)) / (float) segment_height;
        Vec2i A = p0 + (p2 - p0) * alpha;
        Vec2i B = second_half ? p1 + (p2 - p1) * beta : p0 + (p1 - p0) * beta;
        if (A.x > B.x) std::swap(A, B);
        for (int x = A.x; x <= B.x; ++x) {
            image.set(x, p0.y + y, color);
        }
    }
}

// get barycentric
Vec3f barycentric(Vec2i *pts, const Vec2i& P) {
    Vec3i x(pts[1].x - pts[0].x, pts[2].x - pts[0].x, pts[0].x - P.x);
    Vec3i y(pts[1].y - pts[0].y, pts[2].y - pts[0].y, pts[0].y - P.y);
    Vec3i u = x ^ y;

    // u.z is Int, so "abs(u.z) < 1" means "u.z == 0"
    // if u.z is 0, then triangle is degenerate
    if (std::abs(u.z) < 1) return {-1, 1, 1};

    return {1.0f - (float)(u.x + u.y) / (float)u.z,
            (float)u.x / (float)u.z,
            (float)u.y / (float)u.z};
}

bool inside_triangle_cross_product(Vec2i *pts, const Vec2i& P) {
    Vec2i AB(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
    Vec2i BC(pts[2].x - pts[1].x, pts[2].y - pts[1].y);
    Vec2i CA(pts[0].x - pts[2].x, pts[0].y - pts[2].y);

    Vec2i AP(P.x - pts[0].x, P.y - pts[0].y);
    Vec2i BP(P.x - pts[1].x, P.y - pts[1].y);
    Vec2i CP(P.x - pts[2].x, P.y - pts[2].y);

    int z1 = AB ^ AP;
    int z2 = BC ^ BP;
    int z3 = CA ^ CP;

    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}

// triangle drawing with barycentric
void triangle_barycentric(Vec2i *pts, TGAImage &image, const TGAColor& color) {
    // create bounding box
    Vec2i bbox_min(image.get_width() - 1, image.get_height() - 1);
    Vec2i bbox_max(0, 0);
    Vec2i clamp_min(0, 0);
    Vec2i clamp_max(image.get_width() - 1, image.get_height() - 1);
    for (int i = 0; i < 3; ++i) {
        bbox_min.x = std::max(clamp_min.x, std::min(bbox_min.x, pts[i].x));
        bbox_min.y = std::max(clamp_min.y, std::min(bbox_min.y, pts[i].y));

        bbox_max.x = std::min(clamp_max.x, std::max(bbox_max.x, pts[i].x));
        bbox_max.y = std::min(clamp_max.y, std::max(bbox_max.y, pts[i].y));
    }

    Vec2i P;
    for (P.x = bbox_min.x; P.x <= bbox_max.x; ++P.x) {
        for (P.y = bbox_min.y; P.y <= bbox_max.y ; ++P.y) {
            Vec3f bc = barycentric(pts, P);
            if (bc.x < 0 || bc.y < 0 || bc.z < 0) continue;
            image.set(P.x, P.y, color);
        }
    }
}

// triangle drawing with cross product
void triangle_cross_product(Vec2i *pts, TGAImage &image, const TGAColor& color) {
    // create bounding box
    Vec2i bbox_min(image.get_width() - 1, image.get_height() - 1);
    Vec2i bbox_max(0, 0);
    Vec2i clamp_min(0, 0);
    Vec2i clamp_max(image.get_width() - 1, image.get_height() - 1);
    for (int i = 0; i < 3; ++i) {
        bbox_min.x = std::max(clamp_min.x, std::min(bbox_min.x, pts[i].x));
        bbox_min.y = std::max(clamp_min.y, std::min(bbox_min.y, pts[i].y));

        bbox_max.x = std::min(clamp_max.x, std::max(bbox_max.x, pts[i].x));
        bbox_max.y = std::min(clamp_max.y, std::max(bbox_max.y, pts[i].y));
    }

    Vec2i P;
    for (P.x = bbox_min.x; P.x <= bbox_max.x; ++P.x) {
        for (P.y = bbox_min.y; P.y <= bbox_max.y ; ++P.y) {
            if (!inside_triangle_cross_product(pts, P)) continue;
            image.set(P.x, P.y, color);
        }
    }
}


int main(int argc, char** argv) {
    if (argc == 2) {
        model = new Model(argv[1]);
    } else {
        model = new Model("../obj/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);

//    // Random Shading
//    for (int i = 0; i < model->nfaces(); ++i) {
//        std::vector<int> face = model->face(i);
//        Vec2i screen_coords[3];
//        for (int j = 0; j < 3; ++j) {
//            Vec3f word_coord = model->vert(face[j]);
//            screen_coords[j] = Vec2i((int)((word_coord.x + 1.0f) * width / 2.0f), (int)((word_coord.y + 1.0f) * height / 2.0f));
//        }
//        triangle_barycentric(screen_coords, image, TGAColor(rand() % 255, rand() % 255, rand() % 255, 255));
//    }

    Vec3f light_dir(0, 0, -1);

    for (int i = 0; i < model->nfaces(); ++i) {
        // load face
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; ++j) {
            // load vert and project to screen
            Vec3f vert = model->vert(face[j]);
            world_coords[j] = vert;
            screen_coords[j] = Vec2i((int)((vert.x + 1.0f) * width / 2.0f), (int)((vert.y + 1.0f) * height / 2.0f));
        }
        // calculate normal of face
        Vec3f normal = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
        normal.normalize();

        float intensity = normal * light_dir;
        if (intensity > 0)
            triangle_barycentric(screen_coords, image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
    }

    image.flip_vertically();
    //image.write_tga_file("../image/output.tga");
    image.write_tga_file("../image/african_head_simple_light_shading.tga");
    return 0;
}