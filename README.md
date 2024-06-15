# Graphics - TinyRenderer

This is the final project for Computer Graphics course, a tiny software renderer based on [tinyrenderer](https://github.com/ssloy/tinyrenderer/wiki).

My goal is to create a tiny and flexible renderer core that does not rely on any third-party libraries,
which can be a good practice for learning computer graphics.

## Architecture

[model.h](model.h) defines the `Model` class, which can read `.obj` model files and store the vertices and faces in it.

## Features / Algorithms

### Line Drawing

Implement of Bresenham's line algorithm, which can draw a line from p0 to p1.

```c++
// Bresenham's line algorithm
void Renderer::draw_line(Vec2 p0, Vec2 p1, const Color &Color) {
    // if (dx < dy) the line is steep
    // we need to sample it along y axis
    // but we can also get_transpose it, then we can still use x axis
    // just remember to re-get_transpose it when drawing it
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y)) {
        // get_transpose
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    // make it left−to−right
    if (p0.x > p1.x) {
        std::swap(p0, p1);
    }

    // use error to approximate the distance between the line and the pixel, which can improve performance
    int x0 = std::floor(p0.x), y0 = std::floor(p0.y);
    int x1 = std::floor(p1.x), y1 = std::floor(p1.y);
    int dx = x1 - x0;
    int dy = y1 - y0;
    int per_error = std::abs(dy) * 2;
    int error = 0;
    int y = y0;
    for (int x = x0; x <= p1.x; x++) {
        if (steep)
            set_pixel(y, x, Color); // if transposed, de−get_transpose
        else
            set_pixel(x, y, Color);

        error += per_error;
        if (error > dx) {
            y += (p1.y > p0.y ? 1 : -1);
            error -= 2 * dx;
        }
    }
}
```

### Triangle Drawing

#### Line Sweeping

A traditional way to draw a triangle, but it is an old-school approach designed for mono-thread CPU programming, which is not used so much.

*The following line sweeping algorithm is based on Bresenham's line drawing algorithm.*

```c++
// line sweeping triangle drawing
void Renderer::draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, const Color &Color) {
    if (p0.y == p1.y && p1.y == p2.y) return;
    // make p0.y < p1.y < p2.y
    if (p0.y > p1.y) std::swap(p0, p1);
    if (p0.y > p2.y) std::swap(p0, p2);
    if (p1.y > p2.y) std::swap(p1, p2);

    double total_height = p2.y - p0.y;
    for (int y = 0; y <= total_height; ++y) {
        bool second_half = y > p1.y - p0.y || p0.y == p1.y;
        double segment_height = second_half ? p2.y - p1.y : p1.y - p0.y;
        double alpha = static_cast<double>(y) / total_height;
        double beta = static_cast<double>(y - (second_half ? p1.y - p0.y : 0)) / segment_height;
        Vec2 A = p0 + (p2 - p0) * alpha;
        Vec2 B = second_half ? p1 + (p2 - p1) * beta : p0 + (p1 - p0) * beta;
        if (A.x > B.x) std::swap(A, B);
        for (int x = static_cast<int>(A.x); x <= B.x; ++x) {
            set_pixel(x, static_cast<int>(p0.y + y), Color);
        }
    }
}
```

#### Rasterization

Iterate through all pixels of a bounding box and check whether a point belongs a 2D triangle.

There are two ways to check whether a point belongs a 2D triangle.

- Barycentric

[Barycentric](https://en.wikipedia.org/wiki/Barycentric_coordinate_system) is truly a useful feature of triangle which can be used to interpolation. Here, we just use it to check a point.

```c++
// get barycentric
Vec3 Renderer::get_barycentric2D(const Vec2 *t, const Vec2 &p) {
    Mat<3, 3> ABC = {{embed<3>(t[0]), embed<3>(t[1]), embed<3>(t[2])}};
    if (std::abs(ABC.get_det()) < 1e-3) return {-1, 1, 1}; // degenerate check
    return ABC.get_invert().get_transpose() * embed<3>(p);
}
```

- Cross Product

We can also use cross product to check a point.
Just take the cross product of each side of the triangle with the line connecting each vertex to the given point. 

If the results all have the same sign, then the point is inside the triangle.

```c++
// judge by cross product (like GAMES101)
bool Renderer::is_inside_triangle_cross_product(Vec2 *t, const Vec2 &P) {
    Vec2 AB = t[1] - t[0];
    Vec2 BC = t[2] - t[1];
    Vec2 CA = t[0] - t[2];
    Vec2 AP = P - t[0];
    Vec2 BP = P - t[1];
    Vec2 CP = P - t[2];
    double z1 = cross(AB, AP);
    double z2 = cross(BC, BP);
    double z3 = cross(CA, CP);
    return (z1 > 0 && z2 > 0 && z3 >0) || (z1 < 0 && z2 < 0 && z3 < 0);
}
```

- Draw

Then we can draw the triangle with the following steps.

1. find the bounding box (accelerate the calculation)
2. iterate all pixels in the bounding box
3. judge whether the pixel is in the given triangle
4. if not, continue; if yes, fill the pixel with the given Color

*In this case, I use barycentric to check the point.
If you want to use cross product, you just need to change the last if*

```c++
// triangle drawing with barycentric
void Renderer::draw_triangle(const Vec2 *t, const Color &Color) {
    // create bounding box
    int bbox_min[2] = {width_ - 1, height_ - 1};
    int bbox_max[2] = {0, 0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j) {
            bbox_min[j] = std::min(bbox_min[j], static_cast<int>(t[i][j]));
            bbox_max[j] = std::max(bbox_max[j], static_cast<int>(t[i][j]));
        }
    // Ensure the bounding box is within the image boundaries
    bbox_min[0] = std::max(0, bbox_min[0]);
    bbox_min[1] = std::max(0, bbox_min[1]);
    bbox_max[0] = std::min(width_ - 1, bbox_max[0]);
    bbox_max[1] = std::min(height_ - 1, bbox_max[1]);

    for (int x = bbox_min[0]; x <= bbox_max[0]; ++x) {
        for (int y = bbox_min[1]; y <= bbox_max[1]; ++y) {
            Vec3 bc = get_barycentric2D(t, {static_cast<double>(x), static_cast<double>(y)});
            if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                set_pixel(x, y, Color);
            }
        }
    }
}
```

Actually, there are some problems with the fourth step.
I am just simply fill all the pixels with the same given Color, which is only able to render solid-colored triangles.

We can use the interpolation or some other algorithm to create better img.
I will implement it later.

### Shading

#### Random Shading

Just give all triangle random Color. A simple approach to testing.

```c++
// Random Shading
for (int i = 0; i < model->n_faces(); ++i) {
    std::vector<int> face = model->face(i);
    Vec2i screen_coords[3];
    for (int j = 0; j < 3; ++j) {
        Vec3f word_coord = model->vert(face[j]);
        screen_coords[j] = Vec2i((int)((word_coord.x + 1.0f) * width / 2.0f), (int)((word_coord.y + 1.0f) * height / 2.0f));
    }
    draw_triangle(screen_coords, image, Color(rand() % 255, rand() % 255, rand() % 255, 255));
}
```

#### Simple Light Shading

We assume that the intensity of illumination is equal to the scalar product of the light vector and the normal to the given triangle.
The normal to the triangle can be calculated simply as the cross product of its two sides.

Then we

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
        draw_triangle(screen_coords, image, Color(intensity * 255, intensity * 255, intensity * 255, 255));
}
```