#ifndef GRAPHICS_TINYRENDERER_RENDERER_H
#define GRAPHICS_TINYRENDERER_RENDERER_H

#include <cstdint>
#include <vector>
#include "geometry.h"
#include "color.h"

class Renderer {
public:
    enum ColorFormat { GRAYSCALE = 1, RGB = 3, RGBA = 4 };

    Renderer() = default;
    Renderer(int width, int height, int bbp);

    Color get_pixel(int x, int y) const;
    void set_pixel(int x, int y, const Color &color);
    void draw_line(Vec2 p0, Vec2 p1, const Color &color);
    void draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, const Color &color);
    void draw_triangle_barycentric(const Triangle &t, const Color &color);

    void flip_horizontally();
    void flip_vertically();

    int width() const { return width_; }
    int height() const { return height_; }
    std::uint8_t bpp() const { return bpp_; }
    auto frame_data() const { return frame_buffer_.data(); }
    auto& frame_buffer() { return frame_buffer_; }
    auto depth_data() const { return depth_buffer_.data(); }
    auto& depth_buffer() { return depth_buffer_; }
private:
    static Vec3 get_barycentric(const Triangle &t, const Vec2& p);
    static bool is_inside_triangle_cross_product(Vec2 *t, const Vec2& P);

    int width_ = 0;
    int height_ = 0;
    std::uint8_t bpp_ = 0; // bits per pixel
    std::vector<std::uint8_t> frame_buffer_ = {};
    std::vector<double> depth_buffer_ = {};
};

#endif //GRAPHICS_TINYRENDERER_RENDERER_H