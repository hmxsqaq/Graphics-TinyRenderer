#ifndef GRAPHICS_TINYRENDERER_GEOMETRY_H
#define GRAPHICS_TINYRENDERER_GEOMETRY_H

#include <cmath>
#include <cassert>
#include <iostream>

// Vector
template<int N> struct Vec {
    double data[N] = {0};

    double& operator[](const int i)       { assert(i>=0 && i < N); return data[i]; }
    double  operator[](const int i) const { assert(i>=0 && i < N); return data[i]; }
    double  norm2()                 const { return *this * *this; }
    double  norm()                  const { return std::sqrt(norm2()); }
};

template<int N> double operator*(const Vec<N>& vec1, const Vec<N>& vec2) {
    double ret = 0;
    for (int i = 0; i < N; i++) ret += vec1[i] * vec2[i];
    return ret;
}

template<int N> Vec<N> operator*(const double& val, const Vec<N> &vec) {
    Vec<N> ret = vec;
    for (int i = 0; i < N; i++) ret[i] *= val;
    return ret;
}

template<int N> Vec<N> operator*(const Vec<N>& vec, const double& val) {
    Vec<N> ret = vec;
    for (int i = 0; i < N; i++) ret[i] *= val;
    return ret;
}

template<int N> Vec<N> operator/(const Vec<N>& vec, const double& val) {
    Vec<N> ret = vec;
    for (int i = 0; i < N; i++) ret[i] /= val;
    return ret;
}

template<int N> Vec<N> operator+(const Vec<N>& vec1, const Vec<N>& vec2) {
    Vec<N> ret = vec1;
    for (int i = 0; i < N; i++) ret[i] += vec2[i];
    return ret;
}

template<int N> Vec<N> operator-(const Vec<N>& vec1, const Vec<N>& vec2) {
    Vec<N> ret = vec1;
    for (int i = 0; i < N; i++) ret[i] -= vec2[i];
    return ret;
}

template<int N1,int N2> Vec<N1> embed(const Vec<N2> &vec, double fill = 1) {
    Vec<N1> ret;
    for (int i = 0; i < N1; i++) ret[i] = (i < N2 ? vec[i] : fill);
    return ret;
}

template<int N1,int N2> Vec<N1> project(const Vec<N2> &vec) {
    Vec<N1> ret;
    for (int i = 0; i < N1; i++) ret[i] = vec[i];
    return ret;
}

template<int N> std::ostream& operator<<(std::ostream& out, const Vec<N>& vec) {
    for (int i = 0; i < N; i++) out << vec[i] << " ";
    return out;
}

template<> struct Vec<2> {
    double x, y;
    double& operator[](const int i)       { assert(i >= 0 && i < 2); return i ? y : x; }
    double  operator[](const int i) const { assert(i >= 0 && i < 2); return i ? y : x; }
    double  norm2()                 const { return *this * *this; }
    double  norm()                  const { return std::sqrt(norm2()); }
    Vec<2>  normalize()             const { return (*this) / norm(); }
};

template<> struct Vec<3> {
    double x, y, z;
    double& operator[](const int i)       { assert(i >= 0 && i < 3); return i ? (i == 1 ? y : z) : x; }
    double  operator[](const int i) const { assert(i >= 0 && i < 3); return i ? (i == 1 ? y : z) : x; }
    double  norm2()                 const { return *this * *this; }
    double  norm()                  const { return std::sqrt(norm2()); }
    Vec<3>  normalize()             const { return (*this) / norm(); }
};

typedef Vec<2> Vec2;
typedef Vec<3> Vec3;
typedef Vec<4> Vec4;

template<int N>
double dot  (const Vec<N> &v1, const Vec<N> &v2) { return v1 * v2; }
double cross(const Vec2 &v1,   const Vec2 &v2);
Vec3   cross(const Vec3 &v1,   const Vec3 &v2);

// Matrix
template<int N> struct Det;

template<int ROW, int COL> struct Mat {
    Vec<COL> rows[ROW] = {{}};

    Vec<COL>&       operator[](const int i)       { assert(i >= 0 && i < ROW); return rows[i]; }
    const Vec<COL>& operator[](const int i) const { assert(i >= 0 && i < ROW); return rows[i]; }

    Vec<ROW> get_col(const int idx) const {
        assert(idx >= 0 && idx < COL);
        Vec<ROW> ret;
        for (int i = 0; i < ROW; i++) ret[i] = rows[i][idx];
        return ret;
    }

    void set_col(const int idx, const Vec<ROW> &v) {
        assert(idx >= 0 && idx < COL);
        for (int i = 0; i < ROW; i++) rows[i][idx] = v[i];
    }

    // get identity matrix(单位矩阵)
    static Mat<ROW, COL> identity() {
        Mat<ROW, COL> ret;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                ret[i][j] = (i == j);
        return ret;
    }

    // get transpose matrix(转置矩阵)
    Mat<COL, ROW> get_transpose() const {
        Mat<COL, ROW> ret;
        for (int i = 0; i < COL; i++)
            for (int j = 0; j < ROW; j++)
                ret[i][j] = (*this)[j][i];
        return ret;
    }

    // get determinant(行列式)
    double get_det() const {
        return Det<COL>::det(*this);
    }

    // get minor matrix(代数余子式矩阵)
    Mat<ROW - 1, COL - 1> get_minor(const int row, const int col) const {
        Mat<ROW - 1, COL - 1> ret;
        for (int i = 0; i < ROW - 1; i++)
            for (int j = 0; j < COL - 1; j++)
                ret[i][j] = rows[i < row ? i : i + 1][j < col ? j : j + 1];
        return ret;
    }

    // get cofactor(代数余子式)
    double get_cofactor(const int row, const int col) const {
        return get_minor(row, col).get_det() * ((row + col) % 2 ? -1 : 1); // recursion
    }

    // get adjugate matrix(伴随矩阵)
    Mat<COL, ROW> get_adjugate() const {
        Mat<ROW, COL> ret;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                ret[i][j] = get_cofactor(i,j);
        return ret.get_transpose();
    }

    // get invert matrix
    Mat<ROW, COL> get_invert() const {
        Mat<ROW, COL> ret = get_adjugate();
        return ret / (ret.get_col(0) * rows[0]);
    }
};

template<int ROW, int COL> Mat<ROW, COL> operator*(const Mat<ROW, COL>& mat, const double& val) {
    Mat<ROW, COL> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] * val;
    return ret;
}

template<int ROW, int COL> Mat<ROW, COL> operator*(const double& val, const Mat<ROW, COL>& mat) {
    Mat<ROW, COL> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] * val;
    return ret;
}

template<int ROW, int COL> Vec<ROW> operator*(const Mat<ROW, COL>& mat, const Vec<COL>& vec) {
    Vec<ROW> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] * vec;
    return ret;
}

// [ROW,MID] * [MID*COL]
template<int ROW, int MID, int COL> Mat<ROW, COL> operator*(const Mat<ROW, MID>& mat1, const Mat<MID, COL>& mat2) {
    Mat<ROW, COL> ret;
    for (int i = 0; i < ROW; ++i)
        for (int j = 0; j < COL; ++j)
            ret[i][j] = mat1[i] * mat2.get_col(j);
    return ret;
}

template<int ROW,int COL> Mat<ROW,COL> operator/(const Mat<ROW,COL>& mat, const double& val) {
    Mat<ROW,COL> ret;
    for (int i = 0; i < ROW; i++) ret[i] = mat[i] / val;
    return ret;
}

template<int ROW,int COL> Mat<ROW,COL> operator+(const Mat<ROW,COL>& mat1, const Mat<ROW,COL>& mat2) {
    Mat<ROW,COL> ret;
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            ret[i][j] = mat1[i][j] + mat2[i][j];
    return ret;
}

template<int ROW,int COL> Mat<ROW,COL> operator-(const Mat<ROW,COL>& mat1, const Mat<ROW,COL>& mat2) {
    Mat<ROW,COL> ret;
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            ret[i][j] = mat1[i][j] - mat2[i][j];
    return ret;
}

template<int ROW, int COL> std::ostream& operator<<(std::ostream& out, const Mat<ROW, COL>& mat) {
    for (int i = 0; i < ROW; i++) out << mat[i] << std::endl;
    return out;
}

template<int N> struct Det {
    static double det(const Mat<N, N>& src) { // recursion
        double ret = 0;
        for (int i = 0; i < N; i++) ret += src[0][i] * src.get_cofactor(0, i);
        return ret;
    }
};

template<> struct Det<1> {
    static double det(const Mat<1, 1>& src) { // recursion stop point
        return src[0][0];
    }
};

#endif //GRAPHICS_TINYRENDERER_GEOMETRY_H