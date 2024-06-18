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
    const Model& model;
    Vec3 position = {0, 0, 10};
    double angle = 0.0;
    double scale = 1.0;
};

struct Light {
    Vec3 direction;
    Vec3 intensity;
};

struct IShader {
    explicit IShader(const Model &model, std::vector<Light>&& lights = std::vector<Light>())
        : model_(model), lights_(std::move(lights)) {
    }

    virtual void start() { }
    virtual void vertex(int i_face, int nth_vert, Vec4 &ret_vert) = 0;
    virtual bool fragment(const Vec3 &bc, Color& ret_color) = 0;

protected:
    const Model& model_;
    std::vector<Light> lights_;
};

#endif //GRAPHICS_TINYRENDERER_SHADER_H
