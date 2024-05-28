//
// Created by wzh56 on 2024/5/22.
//

#ifndef GRAPHICS_TINYRENDERER_MODEL_H
#define GRAPHICS_TINYRENDERER_MODEL_H

#include <vector>
#include "geometry.h"

class Model {
public:
    explicit Model(const char *filename);
    ~Model();
    Vec3f vert(int idx);
    std::vector<int> face(int idx);

    int n_verts() const;
    int n_faces() const;

private:
    std::vector<Vec3f> verts_;
    std::vector<std::vector<int>> faces_;
};


#endif //GRAPHICS_TINYRENDERER_MODEL_H
