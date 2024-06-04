# Graphics - TinyRenderer

This is the final project for Computer Graphics course, a tiny software renderer based on [tinyrenderer](https://github.com/ssloy/tinyrenderer/wiki).

It does not rely on any third-party libraries, which can be a good practice for learning computer graphics.

## Line Drawing

Implement of Bresenham's line algorithm

```c++
// Bresenham's line algorithm
void line(int x0, int y0, int x1, int y1, TGAImage &image, Color color) {
    // if (dx < dy) the line is steep
    // we need to sample it along y axis
    // but we can also get_transpose it, then we can still use x axis
    // just remember to re-get_transpose it when drawing it
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
            image.set(y, x, color); // if transposed, de−get_transpose
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
void triangle(Vec2i p0, Vec2i p1, Vec2i p2, TGAImage &image, const Color& color) {
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

We can also use cross product to check a point. Just take the cross product of each side of the triangle with the line connecting each vertex to the given point. 

If the results all have the same sign, then the point is inside the triangle.

```c++
bool inside_triangle_cross_product(Vec2i *pts, const Vec2i& P) {
    Vec2i AB(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
    Vec2i BC(pts[2].x - pts[1].x, pts[2].y - pts[1].y);
    Vec2i CA(pts[0].x - pts[2].x, pts[0].y - pts[2].y);

    Vec2i AP(P.x - pts[0].x, P.y - pts[0].y);
    Vec2i BP(P.x - pts[1].x, P.y - pts[1].y);
    Vec2i CP(P.x - pts[2].x, P.y - pts[2].y);

    int z1 = AB ^ AP;
    int z2 = BC ^ BP;
    int z3 = CA ^ CP;

    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}
```

- Draw

Then, we can draw the triangle.

```c++
// triangle drawing with barycentric
void draw_triangle_barycentric(Vec2i *pts, TGAImage &image, const Color& color) {
    // create bounding box
    Vec2i bbox_min(image.get_width() - 1, image.get_height() - 1);
    Vec2i bbox_max(0, 0);
    Vec2i clamp_min(0, 0);
    Vec2i clamp_max(image.get_width() - 1, image.get_height() - 1);
    for (int i = 0; i < 3; ++i) {
        bbox_min.x = std::max(clamp_min.x, std::min(bbox_min.x, pts[i].x));
        bbox_min.y = std::max(clamp_min.y, std::min(bbox_min.y, pts[i].y));

        bbox_max.x = std::min(clamp_max.x, std::max(bbox_max.x, pts[i].x));
        bbox_max.y = std::min(clamp_max.y, std::max(bbox_max.y, pts[i].y));
    }

    Vec2i P;
    for (P.x = bbox_min.x; P.x <= bbox_max.x; ++P.x) {
        for (P.y = bbox_min.y; P.y <= bbox_max.y ; ++P.y) {
            Vec3f bc = barycentric(pts, P);
            if (bc.x < 0 || bc.y < 0 || bc.z < 0) continue;
            image.set(P.x, P.y, color);
        }
    }
}
```

*In this case, I use barycentric to check the point. If you want to use cross product, you just need to change the last if*

## Shading

- Random Shading

Just all random.

```c++
// Random Shading
for (int i = 0; i < model->n_faces(); ++i) {
    std::vector<int> face = model->face(i);
    Vec2i screen_coords[3];
    for (int j = 0; j < 3; ++j) {
        Vec3f word_coord = model->vert(face[j]);
        screen_coords[j] = Vec2i((int)((word_coord.x + 1.0f) * width / 2.0f), (int)((word_coord.y + 1.0f) * height / 2.0f));
    }
    draw_triangle_barycentric(screen_coords, image, Color(rand() % 255, rand() % 255, rand() % 255, 255));
}
```

- Simple Light Shading

We assume that the intensity of illumination is equal to the scalar product of the light vector and the normal to the given triangle. The normal to the triangle can be calculated simply as the cross product of its two sides.

```c++
Vec3f light_dir(0, 0, -1);

for (int i = 0; i < model->n_faces(); ++i) {
    // load face
    std::vector<int> face = model->face(i);
    Vec2i screen_coords[3];
    Vec3f world_coords[3];
    for (int j = 0; j < 3; ++j) {
        // load vert and project to screen
        Vec3f vert = model->vert(face[j]);
        world_coords[j] = vert;
        screen_coords[j] = Vec2i((int)((vert.x + 1.0f) * width / 2.0f), (int)((vert.y + 1.0f) * height / 2.0f));
    }
    // calculate normal of face
    Vec3f normal = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
    normal.normalize();

    float intensity = normal * light_dir;
    if (intensity > 0)
        draw_triangle_barycentric(screen_coords, image, Color(intensity * 255, intensity * 255, intensity * 255, 255));
}
```