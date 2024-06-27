#ifndef GRAPHICS_TINYRENDERER_RENDERER_H
#define GRAPHICS_TINYRENDERER_RENDERER_H

#include <cstdint>
#include <array>
#include <utility>
#include <vector>
#include <omp.h>
#include "color.h"
#include "geometry.h"
#include "shader.h"

void set_model_mat(double angle, double scale, const Vec3 &translate);
void set_view_mat(const Vec3& eye_point);
void set_projection_mat(double fov, double aspect_ratio, double zNear, double zFar);
void set_viewport_mat(int x, int y, int w, int h);

class Renderer {
public:
    Renderer() = default;
    Renderer(int width, int height, int bbp);

    Color get_pixel(int x, int y) const;
    void set_pixel(int x, int y, const Color &color);
    void draw_line(Vec2 p0, Vec2 p1, const Color &color);
    void draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, const Color &color);
    void draw_object(const Object &object, IShader &shader);

    int width() const { return width_; }
    int height() const { return height_; }
    std::uint8_t bpp() const { return bpp_; }
    auto frame_data() const { return frame_buffer_.data(); }
    auto& frame_buffer() { return frame_buffer_; }
    auto depth_data() const { return depth_buffer_.data(); }
    auto& depth_buffer() { return depth_buffer_; }
private:
    void draw_triangle(const Mat<3, 4> &t_vert_clip, IShader &shader);

    static Vec3 get_barycentric2D(const std::array<Vec2, 3> &t_vert, const Vec2& p);
    static bool is_inside_triangle_cross_product(const Vec2 *t, const Vec2& P);

    int width_ = 0;
    int height_ = 0;
    std::uint8_t bpp_ = 0; // bits per pixel
    std::vector<std::uint8_t> frame_buffer_ = {};
    std::vector<double> depth_buffer_ = {};
};

#endif //GRAPHICS_TINYRENDERER_RENDERER_H