//
// Created by wzh56 on 2024/5/22.
//

#ifndef GRAPHICS_TINYRENDERER_GEOMETRY_H
#define GRAPHICS_TINYRENDERER_GEOMETRY_H

#include <cmath>
#include <ostream>

// Vector 2D
template <class T> struct Vec2 {
    union {
        struct {T u, v;};
        struct {T x, y;};
        T raw[2];
    };
    Vec2() : u(0), v(0) { }
    Vec2(T _u, T _v) : u(_u), v(_v) { }

    inline T       operator ^(const Vec2<T> &V) const { return x * V.y - y * V.x;}
    inline Vec2<T> operator +(const Vec2<T> &V) const { return Vec2<T>(u + V.u, v + V.v); }
    inline Vec2<T> operator -(const Vec2<T> &V) const { return Vec2<T>(u - V.u, v - V.v); }
    inline Vec2<T> operator *(float f)          const { return Vec2<T>(u * f, v * f); }
    inline T operator *(const Vec2<T> &V)       const { return x * V.x + y * V.y;}
    template <class > friend std::ostream& operator<<(std::ostream& s, Vec2<T>& v);
};

template <class T> std::ostream& operator<<(std::ostream& s, Vec2<T>& v) {
    s << "(" << v.x << ", " << v.y << ")\n";
    return s;
}

typedef Vec2<float> Vec2f;
typedef Vec2<int> Vec2i;

// Vector 3D
template <class T> struct Vec3 {
    union {
        struct {T x, y, z;};
        struct {T ivert, iuv, inorm;};
        T raw[3];
    };
    Vec3() : x(0), y(0), z(0) { }
    Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) { }

    inline Vec3<T> operator ^(const Vec3<T> &V) const { return Vec3<T>(y * V.z - z * V.y, z * V.x - x * V.z, x * V.y - y * V.x); }
    inline Vec3<T> operator +(const Vec3<T> &V) const { return Vec3<T>(x + V.x, y + V.y, z + V.z); }
    inline Vec3<T> operator -(const Vec3<T> &V) const { return Vec3<T>(x - V.x, y - V.y, z - V.z); }
    inline Vec3<T> operator *(float f)          const { return Vec3<T>(x * f, y * f, z * f); }
    inline T       operator *(const Vec3<T> &V) const { return x * V.x + y * V.y + z * V.z; }
    float norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3<T> & normalize(T l = 1) { *this = (*this) * (l / norm()); return *this; }
    template <class > friend std::ostream& operator<<(std::ostream& s, Vec3<T>& v);
};

template <class T> std::ostream& operator<<(std::ostream& s, Vec3<T>& v) {
    s << "(" << v.x << ", " << v.y << ", " << v.z << ")\n";
    return s;
}

typedef Vec3<float> Vec3f;
typedef Vec3<int> Vec3i;

#endif //GRAPHICS_TINYRENDERER_GEOMETRY_H