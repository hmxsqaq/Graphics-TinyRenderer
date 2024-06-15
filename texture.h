#ifndef GRAPHICS_TINYRENDERER_TEXTURE_H
#define GRAPHICS_TINYRENDERER_TEXTURE_H

#include <cstdint>
#include <vector>
#include "algorithm"
#include "color.h"

struct Texture {
    int width = 0;
    int height = 0;
    std::uint8_t bpp = 0;
    std::vector<std::uint8_t> data = {};

    Texture() = default;
    Texture(int _width, int _height, std::uint8_t _bpp)
        : width(_width), height(_height), bpp(_bpp), data(_width * _height * _bpp) {}

    Color get_pixel(int x, int y) const {
        if (data.empty() || x < 0 || y < 0 || x >= width || y >= height) {
            std::cerr << "get pixel fail: x " << x << " y " << y << "\n";
            return {};
        }

        Color ret = {0, 0, 0, 0};
        const std::uint8_t *pixel = data.data() + (x + y * width) * bpp;
        std::copy_n(pixel, bpp, ret.bgra.begin());
        return ret;
    }

    void flip_horizontally() {
        int half = width >> 1;
        for (int x = 0; x < half; ++x)
            for (int y = 0; y < height; ++y)
                for (int b = 0; b < bpp; ++b)
                    std::swap(data[(x + y * width) * bpp + b],
                              data[(width - 1 - x + y * width) * bpp + b]);
    }

    void flip_vertically() {
        int half = height >> 1;
        for (int x = 0; x < width; ++x)
            for (int y = 0; y < half; ++y)
                for (int b = 0; b < bpp; ++b)
                    std::swap(data[(x + y * width) * bpp + b],
                              data[(x + (height - 1 - y) * width) * bpp + b]);
    }
};

#endif //GRAPHICS_TINYRENDERER_TEXTURE_H
