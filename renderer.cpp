#include "renderer.h"

Mat<4, 4> ModelViewMatrix;
Mat<4, 4> ProjectionMatrix;
Mat<4, 4> ViewportMatrix;

void set_model_mat(double angle, double scale, Vec3 translate) {
    angle = angle * M_PI / 180.0;
    Mat<4, 4> rotation_mat({{{
        {cos(angle), 0, sin(angle), 0},
        {0, 1, 0, 0},
        {-sin(angle), 0, cos(angle), 0},
        {0, 0, 0, 1}
    }}});

    Mat<4, 4> scale_mat({{{
        {scale, 0, 0, 0},
        {0, scale, 0, 0},
        {0, 0, scale, 0},
        {0, 0, 0, 1}
    }}});

    Mat<4, 4> translate_mat({{{
        {1, 0, 0, translate.x},
        {0, 1, 0, translate.y},
        {0, 0, 1, translate.z},
        {0, 0, 0, 1}
    }}});

    ModelViewMatrix = translate_mat * rotation_mat * scale_mat;
}

void set_view_mat(const Vec3 &eye_point) {
    Mat<4, 4> view_matrix = {{{
        {1, 0, 0, -eye_point.x},
        {0, 1, 0, -eye_point.y},
        {0, 0, 1, -eye_point.z},
        {0, 0, 0, 1}
    }}};

    ModelViewMatrix = view_matrix * ModelViewMatrix;
}

void set_projection_mat(double fov, double aspect_ratio, double zNear, double zFar) {
    Mat<4, 4> p2o({{{
        {zNear, 0, 0, 0},
        {0, zNear, 0, 0},
        {0, 0, zNear + zFar, -zNear * zFar},
        {0, 0, 1, 0}
    }}});

    double angle = fov / 180.0 * M_PI;
    double t = tan(angle / 2) * zNear;
    double b = -t;
    double r = t * aspect_ratio;
    double l = -r;
    Mat<4, 4> o2c({{{
        {2 / (r - l), 0, 0, -(r + l) / (r - l)},
        {0, 2 / (t - b), 0, -(t + b) / (t - b)},
        {0, 0, 2 / (zNear - zFar), -(zNear + zFar) / (zNear - zFar)},
        {0, 0, 0, 1}
    }}});

    ProjectionMatrix = o2c * p2o;
}

void set_viewport_mat(int x, int y, int w, int h) {
    ViewportMatrix = {{{
        {w / 2.0, 0, 0, x + w / 2.0},
        {0, h / 2.0, 0, y + h / 2.0},
        {0, 0, 0.5, 0.5},
        {0, 0, 0, 1}
    }}};
}

Renderer::Renderer(int width, int height, int bbp)
    : width_(width), height_(height), bpp_(bbp),
      frame_buffer_(width * height * bbp),
      depth_buffer_(width * height, std::numeric_limits<float>::max()) {
}

Color Renderer::get_pixel(const int x, const int y) const {
    if (frame_buffer_.empty() || x < 0 || y < 0 || x >= width_ || y >= height_) {
        std::cerr << "get pixel fail: x " << x << " y " << y << "\n";
        return {};
    }

    Color ret = {0, 0, 0, 0};
    const std::uint8_t *pixel = frame_buffer_.data() + (x + y * width_) * bpp_;
    std::copy_n(pixel, bpp_, ret.bgra.begin());
    return ret;
}

void Renderer::set_pixel(int x, int y, const Color &color) {
    if (frame_buffer_.empty() || x < 0 || y < 0 || x >= width_ || y >= height_) {
        std::cerr << "set pixel fail: x " << x << " y " << y << "\n";
        return;
    }
//    std::copy(color.bgra.begin(), color.bgra.begin() + bpp_, frame_buffer_.begin() + (x + y * width_) * bpp_);
    std::copy_n(color.bgra.begin(), bpp_, frame_buffer_.begin() + (x + y * width_) * bpp_);
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
    if (p0.x > p1.x) std::swap(p0, p1);

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

void Renderer::draw_object(const Object &object, IShader &shader) {
    set_viewport_mat(0, 0, width_, height_);
    for (int i_face = 0; i_face < object.model.n_faces(); i_face++) {
        Mat<3, 4> t_vert_clip = {0};
        for (int nth_vert : {0, 1, 2})
            shader.vertex(i_face, nth_vert, t_vert_clip[nth_vert]);
        draw_triangle(t_vert_clip, shader);
    }
}

// triangle drawing with barycentric
void Renderer::draw_triangle(const Mat<3, 4> &t_vert_clip, IShader &shader) {
    // clip space 2 screen space
    std::array<Vec2, 3> t_vert_screen_vec2 = {
            resize<2>(ViewportMatrix * t_vert_clip[0] / t_vert_clip[0][3]),
            resize<2>(ViewportMatrix * t_vert_clip[1] / t_vert_clip[1][3]),
            resize<2>(ViewportMatrix * t_vert_clip[2] / t_vert_clip[2][3])
    };
    // create bounding box
    int bbox_min[2] = {width_ - 1, height_ - 1};
    int bbox_max[2] = {0, 0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j) {
            bbox_min[j] = std::min(bbox_min[j], static_cast<int>(t_vert_screen_vec2[i][j]));
            bbox_max[j] = std::max(bbox_max[j], static_cast<int>(t_vert_screen_vec2[i][j]));
        }
    // Ensure the bounding box is within the image boundaries
    bbox_min[0] = std::max(0, bbox_min[0]);
    bbox_min[1] = std::max(0, bbox_min[1]);
    bbox_max[0] = std::min(width_ - 1, bbox_max[0]);
    bbox_max[1] = std::min(height_ - 1, bbox_max[1]);

#pragma omp parallel for // omp optimization (I hope so)
    for (int x = bbox_min[0]; x <= bbox_max[0]; x++) {
        for (int y = bbox_min[1]; y <= bbox_max[1]; y++) {
            Vec3 bc_screen = get_barycentric2D(t_vert_screen_vec2, {static_cast<double>(x), static_cast<double>(y)});
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue; // triangle testing

            // inside triangle
            Vec3 bc_clip = {bc_screen.x / t_vert_clip[0][3], bc_screen.y / t_vert_clip[1][3], bc_screen.z / t_vert_clip[2][3]};
            bc_clip = bc_clip / (bc_clip.x + bc_clip.y + bc_clip.z); // perspective correction, make sure bc.x + bc.y + bc.z = 1
            double depth = t_vert_clip[0][2] * bc_clip.x + t_vert_clip[1][2] * bc_clip.y + t_vert_clip[2][2] * bc_clip.z;
            if (depth > depth_buffer_[x + y * width_]) continue; // depth testing

            // depth test passed
            Color color;
            if (!shader.fragment(bc_clip, color)) continue;

            // fragment shader passed
            set_pixel(x, y, color);
            depth_buffer_[x + y * width_] = depth;
        }
    }
}

// get barycentric
Vec3 Renderer::get_barycentric2D(const std::array<Vec2, 3> &t_vert, const Vec2 &p) {
    // calculate by area
    double x0 = t_vert[0][0], y0 = t_vert[0][1];
    double x1 = t_vert[1][0], y1 = t_vert[1][1];
    double x2 = t_vert[2][0], y2 = t_vert[2][1];
    double t_area = x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1);
    if (std::abs(t_area) < 1e-3) return {-1, 1, 1}; // degenerate check
    double u = (p.x * (y1 - y2) + (x2 - x1) * p.y + x1 * y2 - x2 * y1) / t_area;
    double v = (p.x * (y2 - y0) + (x0 - x2) * p.y + x2 * y0 - x0 * y2) / t_area;
    return {u, v, 1.0 - u - v};
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

void Renderer::set_object(const Object &object) {
    set_model_mat(object.angle, object.scale, object.position);
}

void Renderer::set_camera(const Camera &camera) {
    set_view_mat(camera.position);
    set_projection_mat(camera.fov, camera.aspect_ratio, camera.zNear, camera.zFar);
}
