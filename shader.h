#ifndef GRAPHICS_TINYRENDERER_SHADER_H
#define GRAPHICS_TINYRENDERER_SHADER_H

#include <vector>
#include "geometry.h"
#include "model.h"

struct Camera {
    Vec3 position;
    double fov;
    double aspect_ratio;
    double zNear;
    double zFar;

    Camera(const Vec3& position, double fov, double aspect_ratio, double zNear, double zFar)
            : position(position), fov(fov), aspect_ratio(aspect_ratio), zNear(zNear), zFar(zFar) {}
};

struct Object {
    std::vector<Triangle*> triangle_list;
    Vec3 position;
    double angle;
    double scale;

    Object(const Model &model, const Vec3& position, double angle, double scale)
            : position(position), angle(angle), scale(scale) {
        triangle_list.reserve(model.n_faces());
        for (int i_face = 0; i_face < model.n_faces(); ++i_face) {
            auto* t = new Triangle();
            for (int i_vert = 0; i_vert < 3; ++i_vert) {
                t->vert[i_vert] = resize<4>(model.vert(i_face, i_vert), 1);
                t->normal[i_vert] = model.normal(i_face, i_vert);
                t->tex_coords[i_vert] = model.uv(i_face, i_vert);
            }
            triangle_list.emplace_back(t);
        }
    }

    ~Object() {
        for (auto* t : triangle_list) {
            delete t;
        }
    }
};

struct Light {
    Vec3 position;
    Vec3 intensity;
};

struct IShader {
    IShader() = default;
    IShader(std::vector<Light>&& lights) : lights_(std::move(lights)) { }

    void add_light(Light light) { lights_.push_back(light); }

    void set_model(const Model *model) { model_ = model; }

    void unify_light(const Mat<4,4> model_mat) {
        for (auto& light : lights_) {
            light.position = resize<3>(model_mat * resize<4>(light.position, 0));
            light.position.normalize();
        }
    }

    virtual void vertex(Triangle& t) { }
    virtual bool fragment(const Vec3 &bc, Color& fragment_color) {
        // random shader as default
        fragment_color[0] = rand() % 256;
        fragment_color[1] = rand() % 256;
        fragment_color[2] = rand() % 256;
        fragment_color[3] = rand() % 256;
        return true;
    }
protected:
    const Model *model_ = nullptr;
    std::vector<Light> lights_{};
};

struct Shader : public IShader {
    Shader() = default;
    Shader(std::vector<Light>&& lights) : IShader(std::move(lights)) { }

    void vertex(Triangle& t) override {
        color_ = t.color;
        normal_ = t.normal;
        tex_coords_ = t.tex_coords;
        view_vert_ = t.view_vert;
    }

    bool fragment(const Vec3 &bc, Color& fragment_color) override {
        if (model_ == nullptr) {
            std::cerr << "No model loaded" << std::endl;
            return false;
        }
        if (model_->diffuse() == nullptr) {
            std::cerr << "No texture loaded" << std::endl;
            return false;
        }

        auto color = interpolate(bc, color_[0], color_[1], color_[2], 1);
        auto normal = interpolate(bc, normal_[0], normal_[1], normal_[2], 1);
        auto tex_coords = interpolate(bc, tex_coords_[0], tex_coords_[1], tex_coords_[2], 1);
        auto view_vert = interpolate(bc, view_vert_[0], view_vert_[1], view_vert_[2], 1);
        fragment_color = model_->diffuse()->get_color(tex_coords);
        return true;
    }

protected:
    Mat<3, 3> color_{};
    Mat<3, 3> normal_{};
    Mat<3, 2> tex_coords_{};
    Mat<3, 3> view_vert_{};
};

#endif //GRAPHICS_TINYRENDERER_SHADER_H
