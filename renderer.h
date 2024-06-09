#ifndef GRAPHICS_TINYRENDERER_RENDERER_H
#define GRAPHICS_TINYRENDERER_RENDERER_H

#include <cstdint>
#include <vector>
#include "geometry.h"

struct Color {
    std::uint8_t bgra[4] = {0,0,0,0}; // BLUE, GREEN, RED, alpha
    std::uint8_t bytespp = 4; // number of bytes per pixel, 3 for RGB, 4 for RGBA
    std::uint8_t& operator[](const int i) { return bgra[i]; }
};

std::ostream &operator<<(std::ostream &out, const Color &color);

class Renderer {
public:
    enum ColorFormat { GRAYSCALE = 1, RGB = 3, RGBA = 4 };

    Renderer() = default;
    Renderer(int width, int height, int bbp);

    Color get_pixel(int x, int y) const;
    void set_pixel(int x, int y, const Color &color);
    void draw_line(Vec2 p0, Vec2 p1, const Color &color);
    void draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, const Color &color);
    void draw_triangle_barycentric(const Vec2 *t, const Color& color);

    void flip_horizontally();
    void flip_vertically();

    int width() const { return width_; }
    int height() const { return height_; }
    std::uint8_t bpp() const { return bpp_; }
    auto data() const { return frame_buffer_.data(); }
    auto& frame_buffer() { return frame_buffer_; }
private:
    static Vec3 get_barycentric(const Vec2 *t, const Vec2& p);
    // judge by cross product (like GAMES101)
    static bool is_inside_triangle_cross_product(Vec2 *t, const Vec2& P);

    int width_ = 0;
    int height_ = 0;
    std::uint8_t bpp_ = 0; // bits per pixel
    std::vector<std::uint8_t> frame_buffer_ = {};
};

#endif //GRAPHICS_TINYRENDERER_RENDERER_H