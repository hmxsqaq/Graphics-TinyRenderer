# Graphics - TinyRenderer

This is the final project for Computer Graphics course, a tiny software renderer based on [tinyrenderer](https://github.com/ssloy/tinyrenderer/wiki).

It does not rely on any third-party libraries, which can be a good practice for learning computer graphics.

## Line Drawing

Implement of Bresenham's line algorithm

```c++
// Bresenham's line algorithm
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    // if (dx < dy) the line is steep
    // we need to sample it along y axis
    // but we can also transpose it, then we can still use x axis
    // just remember to re-transpose it when drawing it
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) { // make it left−to−right
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    // use error to approximate the distance between the line and the pixel, which can improve performance
    int dx = x1 - x0;
    int dy = y1 - y0;
    int derror = std::abs(dy) * 2;
    int error = 0;
    int y = y0;

    for (int x = x0; x <= x1; x++) {
        if (steep) {
            image.set(y, x, color); // if transposed, de−transpose
        } else {
            image.set(x, y, color);
        }
        error += derror;
        if (error > dx) {
            y += (y1 > y0 ? 1 : -1);
            error -= 2 * dx;
        }
    }
}
```

## Model Loading

[.obj](http://en.wikipedia.org/wiki/Wavefront_.obj_file) is a geometry definition file format which is simple and open.

[model.h](model.h) defines the `Model` class, which can read `.obj` model files and store the vertices and faces in it.

## Triangle Drawing

### Line Sweeping

A traditional way to draw a triangle, but it is an old-school approach designed for mono-thread CPU programming, which is not used so much.

```c++
// line sweeping triangle drawing
void triangle(Vec2i p0, Vec2i p1, Vec2i p2, TGAImage &image, const TGAColor& color) {
    if (p0.y == p1.y && p1.y == p2.y) return;
    // make p0.y < p1.y < p2.y
    if (p0.y > p1.y) std::swap(p0, p1);
    if (p0.y > p2.y) std::swap(p0, p2);
    if (p1.y > p2.y) std::swap(p1, p2);

    int total_height = p2.y - p0.y;
    for (int y = 0; y <= total_height; ++y) {
        bool second_half = y > p1.y - p0.y || p0.y == p1.y;
        int segment_height = second_half ? p2.y - p1.y : p1.y - p0.y;
        float alpha = (float)y / (float)total_height;
        float beta = (float)(y - (second_half ? p1.y - p0.y : 0)) / (float) segment_height;
        Vec2i A = p0 + (p2 - p0) * alpha;
        Vec2i B = second_half ? p1 + (p2 - p1) * beta : p0 + (p1 - p0) * beta;
        if (A.x > B.x) std::swap(A, B);
        for (int x = A.x; x <= B.x; ++x) {
            image.set(x, p0.y + y, color);
        }
    }
}
```

### Rasterization

Iterate through all pixels of a bounding box and check whether a point belongs a 2D triangle.

There are two ways to check whether a point belongs a 2D triangle.

- Barycentric

[Barycentric](https://en.wikipedia.org/wiki/Barycentric_coordinate_system) is truly an useful feature of triangle which can be used to interpolation. Here, we just use it to check a point.

```c++
// get barycentric
Vec3f barycentric(Vec2i *pts, Vec2i P) {
    Vec3i x(pts[1].x - pts[0].x, pts[2].x - pts[0].x, pts[0].x - P.x);
    Vec3i y(pts[1].y - pts[0].y, pts[2].y - pts[0].y, pts[0].y - P.y);
    Vec3i u = x ^ y;

    // u.z is Int, so "abs(u.z) < 1" means "u.z == 0"
    // if u.z is 0, then triangle is degenerate
    if (std::abs(u.z) < 1) return {-1, 1, 1};

    return {1.0f - (float)(u.x + u.y) / (float)u.z,
            (float)u.x / (float)u.z,
            (float)u.y / (float)u.z};
}
```

- Cross Product