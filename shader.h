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
    IShader(const Model *model, std::vector<Light>&& lights) : model_(model), lights_(std::move(lights)) { }

    void add_light(Light light) { lights_.push_back(light); }

    virtual void vertex(Triangle& t) { }
    virtual bool fragment(const Triangle& t, Color& fragment_color) {
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
    Shader(const Model *model, std::vector<Light>&& lights) : IShader(model, std::move(lights)) { }

//    bool fragment(const Triangle& t, Color& fragment_color) override {
//        return true;
//    }
};

#endif //GRAPHICS_TINYRENDERER_SHADER_H
