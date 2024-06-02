#include "geometry.h"

double cross(const Vec2 &v1, const Vec2 &v2) {
    return v1.x * v2.y - v1.y * v2.x;
}

Vec3 cross(const Vec3 &v1, const Vec3 &v2) {
    return Vec<3>{v1.y * v2.z - v1.z * v2.y,
                  v1.z * v2.x - v1.x * v2.z,
                  v1.x * v2.y - v1.y * v2.x};
}