# Graphics - TinyRenderer

This is the final project for Computer Graphics course, a tiny software renderer based on [tinyrenderer](https://github.com/ssloy/tinyrenderer/wiki).

It does not rely on any third-party libraries, which can be a good practice for learning computer graphics.

## Line drawing

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

## Model loading

[.obj](http://en.wikipedia.org/wiki/Wavefront_.obj_file) is a geometry definition file format which is simple and open.

[model.h](model.h) defines the `Model` class, which can read [.obj](http://en.wikipedia.org/wiki/Wavefront_.obj_file) model files and store the vertices and faces in it.