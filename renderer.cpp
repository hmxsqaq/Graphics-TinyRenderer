#include "renderer.h"

std::ostream &operator<<(std::ostream &out, const Color &color) {
    for (unsigned char i : color.bgra) out << (int)i << " ";
    return out;
}

Renderer::Renderer(int width, int height, int bbp)
    : width_(width), height_(height), bpp_(bbp), frame_buffer_(width * height * bbp) {
}

Color Renderer::get_pixel(const int x, const int y) const {
    if (frame_buffer_.empty() || x < 0 || y < 0 || x >= width_ || y >= height_) return {};

    Color ret = {0, 0, 0, 0, bpp_};
    const std::uint8_t *pixel = frame_buffer_.data() + (x + y * width_) * bpp_;
    for (int i = 0; i < bpp_; i++) ret.bgra[i] = pixel[i];
    return ret;
}

void Renderer::set_pixel(int x, int y, const Color &color) {
    if (frame_buffer_.empty() || x < 0 || y < 0 || x >= width_ || y >= height_) return;

    memcpy(frame_buffer_.data() + (x + y * width_) * bpp_, color.bgra, bpp_);
}

// Bresenham's line algorithm
void Renderer::draw_line(Vec2 p0, Vec2 p1, const Color &color) {
    // if (dx < dy) the line is steep
    // we need to sample it along y axis
    // but we can also get_transpose it, then we can still use x axis
    // just remember to re-get_transpose it when drawing it
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y)) {
        // get_transpose
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    // make it left−to−right
    if (p0.x > p1.x) {
        std::swap(p0, p1);
    }

    // use error to approximate the distance between the line and the pixel, which can improve performance
    int x0 = std::floor(p0.x), y0 = std::floor(p0.y);
    int x1 = std::floor(p1.x), y1 = std::floor(p1.y);
    int dx = x1 - x0;
    int dy = y1 - y0;
    int per_error = std::abs(dy) * 2;
    int error = 0;
    int y = y0;
    for (int x = x0; x <= p1.x; x++) {
        if (steep)
            set_pixel(y, x, color); // if transposed, de−get_transpose
        else
            set_pixel(x, y, color);

        error += per_error;
        if (error > dx) {
            y += (p1.y > p0.y ? 1 : -1);
            error -= 2 * dx;
        }
    }
}

// line sweeping triangle drawing
void Renderer::draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, const Color &color) {
    if (p0.y == p1.y && p1.y == p2.y) return;
    // make p0.y < p1.y < p2.y
    if (p0.y > p1.y) std::swap(p0, p1);
    if (p0.y > p2.y) std::swap(p0, p2);
    if (p1.y > p2.y) std::swap(p1, p2);

    double total_height = p2.y - p0.y;
    for (int y = 0; y <= total_height; ++y) {
        bool second_half = y > p1.y - p0.y || p0.y == p1.y;
        double segment_height = second_half ? p2.y - p1.y : p1.y - p0.y;
        double alpha = static_cast<double>(y) / total_height;
        double beta = static_cast<double>(y - (second_half ? p1.y - p0.y : 0)) / segment_height;
        Vec2 A = p0 + (p2 - p0) * alpha;
        Vec2 B = second_half ? p1 + (p2 - p1) * beta : p0 + (p1 - p0) * beta;
        if (A.x > B.x) std::swap(A, B);
        for (int x = static_cast<int>(A.x); x <= B.x; ++x) {
            set_pixel(x, static_cast<int>(p0.y + y), color);
        }
    }
}

// triangle drawing with barycentric
void Renderer::draw_triangle_barycentric(const Vec2 *t, const Color &color) {
    // create bounding box
    int bbox_min[2] = {width_ - 1, height_ - 1};
    int bbox_max[2] = {0, 0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j) {
            bbox_min[j] = std::min(bbox_min[j], static_cast<int>(t[i][j]));
            bbox_max[j] = std::max(bbox_max[j], static_cast<int>(t[i][j]));
        }
    // Ensure the bounding box is within the image boundaries
    bbox_min[0] = std::max(0, bbox_min[0]);
    bbox_min[1] = std::max(0, bbox_min[1]);
    bbox_max[0] = std::min(width_ - 1, bbox_max[0]);
    bbox_max[1] = std::min(height_ - 1, bbox_max[1]);

    for (int x = bbox_min[0]; x <= bbox_max[0]; ++x) {
        for (int y = bbox_min[1]; y <= bbox_max[1]; ++y) {
            Vec3 bc = get_barycentric(t, {static_cast<double>(x), static_cast<double>(y)});
            if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                set_pixel(x, y, color);
            }
        }
    }
}

void Renderer::flip_horizontally() {
    int half = width_ >> 1;
    for (int x = 0; x < half; ++x)
        for (int y = 0; y < height_; ++y)
            for (int b = 0; b < bpp_; ++b)
                std::swap(frame_buffer_[(x + y * width_) * bpp_ + b],
                          frame_buffer_[(width_ - 1 - x + y * width_) * bpp_ + b]);
}

void Renderer::flip_vertically() {
    int half = height_ >> 1;
    for (int x = 0; x < width_; ++x)
        for (int y = 0; y < half; ++y)
            for (int b = 0; b < bpp_; ++b)
                std::swap(frame_buffer_[(x + y * width_) * bpp_ + b],
                          frame_buffer_[(x + (height_ - 1 - y) * width_) * bpp_ + b]);
}

// get barycentric
Vec3 Renderer::get_barycentric(const Vec2 *t, const Vec2 &p) {
    Mat<3, 3> ABC = {{embed<3>(t[0]), embed<3>(t[1]), embed<3>(t[2])}};
    if (std::abs(ABC.get_det()) < 1e-3) return {-1, 1, 1}; // degenerate check
    return ABC.get_invert().get_transpose() * embed<3>(p);
}

// judge by cross product (like GAMES101)
bool Renderer::is_inside_triangle_cross_product(Vec2 *t, const Vec2 &P) {
    Vec2 AB = t[1] - t[0];
    Vec2 BC = t[2] - t[1];
    Vec2 CA = t[0] - t[2];
    Vec2 AP = P - t[0];
    Vec2 BP = P - t[1];
    Vec2 CP = P - t[2];
    double z1 = cross(AB, AP);
    double z2 = cross(BC, BP);
    double z3 = cross(CA, CP);
    return (z1 > 0 && z2 > 0 && z3 >0) || (z1 < 0 && z2 < 0 && z3 < 0);
}