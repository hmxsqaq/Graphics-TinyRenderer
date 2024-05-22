#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);

void Line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    for (float t = 0; t < 1; t += 0.1f) {
        int x = x0 * (1.0f - t) + x1 * t;
        int y = y0 * (1.0f - t) + y1 * t;
        image.set(x, y, color);
    }
}

int main(int argc, char** argv) {
    TGAImage image(100, 100, TGAImage::RGB);
    //image.set(52, 41, white);
    Line(13, 20, 80, 40, image, white);
    image.flip_vertically();
    image.write_tga_file("output.tga");
    return 0;
}