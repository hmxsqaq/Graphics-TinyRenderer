#ifndef GRAPHICS_TINYRENDERER_RENDERER_H
#define GRAPHICS_TINYRENDERER_RENDERER_H

#include <cstdint>
#include <array>
#include <utility>
#include <vector>
#include <functional>
#include <omp.h>
#include "geometry.h"
#include "color.h"
#include "shader.h"

class Model;

class Renderer {
public:
    enum ColorFormat { GRAYSCALE = 1, RGB = 3, RGBA = 4 };

    Renderer() = default;
    Renderer(int width, int height, int bbp);

    Color get_pixel(int x, int y) const;
    void set_pixel(int x, int y, const Color &color);
    void draw_line(Vec2 p0, Vec2 p1, const Color &color);
    void draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, const Color &color);
    void draw_triangle(const Triangle &t, const std::array<Vec3, 3> &view_pos);
    void draw_triangle_list(std::vector<Triangle *> &t_list);

    void set_model_mat(double angle, double scale, Vec3 translate);
    void set_view_mat(const Vec3& eye_point);
    void set_projection_mat(double eye_fov, double aspect_ratio, double zNear, double zFar);

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
    static Vec3 get_barycentric2D(const Triangle &t, const Vec2& p);
    static bool is_inside_triangle_cross_product(Vec2 *t, const Vec2& P);

    int width_ = 0;
    int height_ = 0;
    std::uint8_t bpp_ = 0; // bits per pixel
    std::vector<std::uint8_t> frame_buffer_ = {};
    std::vector<double> depth_buffer_ = {};

    Mat<4, 4> model_mat_ = {};
    Mat<4, 4> view_mat_ = {};
    Mat<4, 4> projection_mat_ = {};
};

#endif //GRAPHICS_TINYRENDERER_RENDERER_H