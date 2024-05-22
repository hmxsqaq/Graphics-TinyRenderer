//
// Created by wzh56 on 2024/5/22.
//

#ifndef GRAPHICS_TINYRENDERER_MODEL_H
#define GRAPHICS_TINYRENDERER_MODEL_H

#include <vector>
#include "geometry.h"

class Model {
private:
    std::vector<Vec3f> verts_;
    std::vector<std::vector<int>> faces_;
public:
    Model(const char *filename);
    ~Model();
    int vertsSize();
    int facesSize();
    Vec3f vert(int idx);
    std::vector<int> face(int idx);
};


#endif //GRAPHICS_TINYRENDERER_MODEL_H
