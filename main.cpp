#include "tgaimage.h"
#include "model.h"

const TGAColor white =  TGAColor(255,   255,    255,    255);
const TGAColor red =    TGAColor(255,   0,      0,      255);
const TGAColor green =  TGAColor(0,     255,    0,      255);

Model *model = nullptr;
const int width  = 200;
const int height = 200;

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

// get barycentric
Vec3f barycentric(Vec2i *pts, Vec2i P) {
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

// line sweeping triangle drawing
void triangle(Vec2i p0, Vec2i p1, Vec2i p2, TGAImage &image, const TGAColor& color) {
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

int main(int argc, char** argv) {
//    if (argc == 2) {
//        model = new Model(argv[1]);
//    } else {
//        model = new Model("../obj/african_head.obj");
//    }

    TGAImage image(width, height, TGAImage::RGB);
    Vec2i t0[3] = {Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80)};
    Vec2i t1[3] = {Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180)};
    Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};
    triangle(t0[0], t0[1], t0[2], image, red);
    triangle(t1[0], t1[1], t1[2], image, white);
    triangle(t2[0], t2[1], t2[2], image, green);

    image.flip_vertically();
    image.write_tga_file("../image/output.tga");
    return 0;
}