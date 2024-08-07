#ifndef GRAPHICS_TINYRENDERER_GEOMETRY_H
#define GRAPHICS_TINYRENDERER_GEOMETRY_H

#include <cmath>
#include <cassert>
#include <iostream>
#include <array>

// Vector
template<int N>
struct Vec {
    std::array<double, N> data;

    constexpr double& operator[](const int i)       noexcept { assert(i>=0 && i < N); return data[i]; }
    constexpr double  operator[](const int i) const noexcept { assert(i>=0 && i < N); return data[i]; }
    constexpr double  norm2()                 const noexcept { return *this * *this; }
    constexpr double  norm()                  const noexcept { return std::sqrt(norm2()); }
};

template<int N>
constexpr double operator*(const Vec<N>& vec1, const Vec<N>& vec2) noexcept {
    double ret = 0;
    for (int i = 0; i < N; i++) ret += vec1[i] * vec2[i];
    return ret;
}

template<int N>
constexpr Vec<N> operator*(const double& val, const Vec<N> &vec) noexcept {
    Vec<N> ret = vec;
    for (int i = 0; i < N; i++) ret[i] *= val;
    return ret;
}

template<int N>
constexpr Vec<N> operator*(const Vec<N>& vec, const double& val) noexcept {
    Vec<N> ret = vec;
    for (int i = 0; i < N; i++) ret[i] *= val;
    return ret;
}

template<int N>
constexpr Vec<N> operator/(const Vec<N>& vec, const double& val) noexcept {
    assert(val != 0);
    Vec<N> ret = vec;
    for (int i = 0; i < N; i++) ret[i] /= val;
    return ret;
}

template<int N>
constexpr Vec<N> operator+(const Vec<N>& vec1, const Vec<N>& vec2) noexcept {
    Vec<N> ret = vec1;
    for (int i = 0; i < N; i++) ret[i] += vec2[i];
    return ret;
}

template<int N>
constexpr Vec<N> operator-(const Vec<N>& vec1, const Vec<N>& vec2) noexcept {
    Vec<N> ret = vec1;
    for (int i = 0; i < N; i++) ret[i] -= vec2[i];
    return ret;
}

template<int N1, int N2>
constexpr Vec<N1> resize(const Vec<N2> &vec, double fill = 1) noexcept {
    Vec<N1> ret{};
    for (int i = 0; i < N1; i++)
        ret[i] = (i < N2 ? vec[i] : fill);
    return ret;
}

template<int N>
constexpr std::ostream& operator<<(std::ostream& out, const Vec<N>& vec) noexcept {
    for (int i = 0; i < N; i++) out << vec[i] << " ";
    return out;
}

template<>
struct Vec<2> {
    double x = 0, y = 0;
    constexpr double& operator[](const int i)       noexcept { assert(i >= 0 && i < 2); return i ? y : x; }
    constexpr double  operator[](const int i) const noexcept { assert(i >= 0 && i < 2); return i ? y : x; }
    constexpr double  norm2()                 const noexcept { return *this * *this; }
    constexpr double  norm()                  const noexcept { return std::sqrt(norm2()); }
    constexpr Vec<2>  normalize()             const noexcept { assert(norm() != 0); return (*this) / norm(); }
};

template<>
struct Vec<3> {
    double x = 0, y = 0, z = 0;
    constexpr double& operator[](const int i)       noexcept { assert(i >= 0 && i < 3); return i ? (i == 1 ? y : z) : x; }
    constexpr double  operator[](const int i) const noexcept { assert(i >= 0 && i < 3); return i ? (i == 1 ? y : z) : x; }
    constexpr double  norm2()                 const noexcept { return *this * *this; }
    constexpr double  norm()                  const noexcept { return std::sqrt(norm2()); }
    constexpr Vec<3>  normalize()             const noexcept { assert(norm() != 0); return (*this) / norm(); }
};

using Vec2 = Vec<2>;
using Vec3 = Vec<3>;
using Vec4 = Vec<4>;

template<int N>
constexpr double dot(const Vec<N> &v1, const Vec<N> &v2) noexcept { return v1 * v2; }
constexpr double cross(const Vec2 &v1, const Vec2 &v2) noexcept { return v1.x * v2.y - v1.y * v2.x; }
constexpr Vec3 cross(const Vec3 &v1, const Vec3 &v2) noexcept {
    return Vec<3>{v1.y * v2.z - v1.z * v2.y,
                  v1.z * v2.x - v1.x * v2.z,
                  v1.x * v2.y - v1.y * v2.x};
}

template<int N>
constexpr Vec<N> interpolate(const Vec3 &bc, const Vec<N> &v1, const Vec<N> &v2, const Vec<N> &v3, double weight) {
    return (bc.x * v1 + bc.y * v2 + bc.z * v3) / weight;
}

// Matrix
template<int N> struct Det;

template<int ROW, int COL>
struct Mat {
    std::array<Vec<COL>, ROW> rows;

    constexpr Vec<COL>&       operator[](const int i)       noexcept { assert(i >= 0 && i < ROW); return rows[i]; }
    constexpr const Vec<COL>& operator[](const int i) const noexcept { assert(i >= 0 && i < ROW); return rows[i]; }

    constexpr Vec<ROW> col(const int idx) const noexcept {
        assert(idx >= 0 && idx < COL);
        Vec<ROW> ret;
        for (int i = 0; i < ROW; i++) ret[i] = rows[i][idx];
        return ret;
    }

    constexpr void set_col(const int idx, const Vec<ROW> &v) noexcept {
        assert(idx >= 0 && idx < COL);
        for (int i = 0; i < ROW; i++) rows[i][idx] = v[i];
    }

    // get identity matrix(单位矩阵)
    constexpr static Mat<ROW, COL> identity() noexcept {
        Mat<ROW, COL> ret;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                ret[i][j] = (i == j);
        return ret;
    }

    // get transpose matrix(转置矩阵)
    constexpr Mat<COL, ROW> transpose() const noexcept {
        Mat<COL, ROW> ret;
        for (int i = 0; i < COL; i++)
            for (int j = 0; j < ROW; j++)
                ret[i][j] = (*this)[j][i];
        return ret;
    }

    // get determinant(行列式)
    constexpr double det() const noexcept {
        return Det<COL>::det(*this);
    }

    // get minor matrix(代数余子式矩阵)
    constexpr Mat<ROW - 1, COL - 1> minor(const int row, const int col) const noexcept {
        Mat<ROW - 1, COL - 1> ret;
        for (int i = 0; i < ROW - 1; i++)
            for (int j = 0; j < COL - 1; j++)
                ret[i][j] = rows[i < row ? i : i + 1][j < col ? j : j + 1];
        return ret;
    }

    // get cofactor(代数余子式)
    constexpr double cofactor(const int row, const int col) const noexcept {
        return minor(row, col).det() * ((row + col) % 2 ? -1 : 1); // recursion
    }

    // get adjugate matrix(伴随矩阵)
    constexpr Mat<COL, ROW> adjugate_transpose() const noexcept {
        Mat<ROW, COL> ret;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                ret[i][j] = cofactor(i, j);
        return ret;
    }

    constexpr Mat<COL, ROW> adjugate() const noexcept {
        return adjugate_transpose().transpose();
    }

    // get invert matrix
    constexpr Mat<ROW, COL> invert_transpose() const {
        Mat<ROW, COL> ret = adjugate_transpose();
        double det = ret[0] * rows[0];
        assert(det != 0);
        return ret / det;
    }

    constexpr Mat<ROW, COL> invert() const {
        return invert_transpose().transpose();
    }
};

template<int ROW, int COL>
constexpr Mat<ROW, COL> operator*(const Mat<ROW, COL>& mat, const double& val) noexcept {
    Mat<ROW, COL> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] * val;
    return ret;
}

template<int ROW, int COL>
constexpr Mat<ROW, COL> operator*(const double& val, const Mat<ROW, COL>& mat) noexcept {
    Mat<ROW, COL> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] * val;
    return ret;
}

template<int ROW, int COL>
constexpr Vec<ROW> operator*(const Mat<ROW, COL>& mat, const Vec<COL>& vec) noexcept {
    Vec<ROW> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] * vec;
    return ret;
}

// [ROW,MID] * [MID*COL]
template<int ROW, int MID, int COL>
constexpr Mat<ROW, COL> operator*(const Mat<ROW, MID>& mat1, const Mat<MID, COL>& mat2) noexcept {
    Mat<ROW, COL> ret;
    for (int i = 0; i < ROW; ++i)
        for (int j = 0; j < COL; ++j)
            ret[i][j] = mat1[i] * mat2.col(j);
    return ret;
}

template<int ROW,int COL>
constexpr Mat<ROW,COL> operator/(const Mat<ROW,COL>& mat, const double& val) noexcept {
    assert(val != 0);
    Mat<ROW,COL> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] / val;
    return ret;
}

template<int ROW,int COL>
constexpr Mat<ROW,COL> operator+(const Mat<ROW,COL>& mat1, const Mat<ROW,COL>& mat2) noexcept {
    Mat<ROW,COL> ret;
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            ret[i][j] = mat1[i][j] + mat2[i][j];
    return ret;
}

template<int ROW,int COL>
constexpr Mat<ROW,COL> operator-(const Mat<ROW,COL>& mat1, const Mat<ROW,COL>& mat2) noexcept {
    Mat<ROW,COL> ret;
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            ret[i][j] = mat1[i][j] - mat2[i][j];
    return ret;
}

template<int ROW, int COL>
constexpr std::ostream& operator<<(std::ostream& out, const Mat<ROW, COL>& mat) noexcept {
    for (int i = 0; i < ROW; i++) out << mat[i] << "\n";
    return out;
}

template<int N>
struct Det {
    constexpr static double det(const Mat<N, N>& src) noexcept { // recursion
        double ret = 0;
        for (int i = 0; i < N; i++) ret += src[0][i] * src.cofactor(0, i);
        return ret;
    }
};

template<> struct Det<1> {
    constexpr static double det(const Mat<1, 1>& src) noexcept { // recursion stop point
        return src[0][0];
    }
};

#endif //GRAPHICS_TINYRENDERER_GEOMETRY_H