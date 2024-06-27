#ifndef GRAPHICS_TINYRENDERER_TEXTURE_H
#define GRAPHICS_TINYRENDERER_TEXTURE_H

#include <cstdint>
#include <vector>
#include "algorithm"
#include "color.h"
#include "geometry.h"

struct Texture {
    int width = 0;
    int height = 0;
    std::uint8_t bpp = 0;
    std::vector<std::uint8_t> data = {};

    Texture() = default;
    Texture(const int _width, const int _height, const std::uint8_t _bpp)
        : width(_width), height(_height), bpp(_bpp), data(_width * _height * _bpp) {}

    Color get_color(const int x, const int y) const {
        if (data.empty() || x < 0 || y < 0 || x >= width || y >= height) {
            std::cerr << "get pixel fail: x " << x << " y " << y << "\n";
            return {};
        }

        Color ret = {0, 0, 0, 0};
        const std::uint8_t *pixel = data.data() + (x + y * width) * bpp;
        std::copy_n(pixel, bpp, ret.bgra.begin());
        return ret;
    }

    Color get_color(const Vec2& uv) const {
        return get_color(static_cast<int>(uv[0] * width), static_cast<int>(uv[1] * height));
    }

    void flip_horizontally() {
        const int half = width >> 1;
        for (int x = 0; x < half; ++x)
            for (int y = 0; y < height; ++y)
                for (int b = 0; b < bpp; ++b)
                    std::swap(data[(x + y * width) * bpp + b],
                              data[(width - 1 - x + y * width) * bpp + b]);
    }

    void flip_vertically() {
        const int half = height >> 1;
        for (int x = 0; x < width; ++x)
            for (int y = 0; y < half; ++y)
                for (int b = 0; b < bpp; ++b)
                    std::swap(data[(x + y * width) * bpp + b],
                              data[(x + (height - 1 - y) * width) * bpp + b]);
    }
};

#endif //GRAPHICS_TINYRENDERER_TEXTURE_H
