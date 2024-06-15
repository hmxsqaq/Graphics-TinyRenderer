#ifndef GRAPHICS_TINYRENDERER_COLOR_H
#define GRAPHICS_TINYRENDERER_COLOR_H

#include <cstdint>
#include <iostream>
#include <cassert>
#include <array>

struct Color {
    enum ColorFormat { GRAYSCALE = 1, RGB = 3, RGBA = 4 };

    std::array<std::uint8_t, 4> bgra = {0, 0, 0, 0}; // BLUE, GREEN, RED, alpha
    // std::uint8_t bpp = 4; // number of bytes per pixel, 3 for RGB, 4 for RGBA
    constexpr std::uint8_t& operator[](const int i) noexcept { assert(i >= 0 && i < 4); return bgra[i]; }
    constexpr std::uint8_t R() const noexcept { return bgra[2]; }
    constexpr std::uint8_t G() const noexcept { return bgra[1]; }
    constexpr std::uint8_t B() const noexcept { return bgra[0]; }
    constexpr std::uint8_t A() const noexcept { return bgra[3]; }
//    friend std::ostream& operator<<(std::ostream &out, const Color &color);
};

inline std::ostream& operator<<(std::ostream &out, const Color &color) {
    out <<
    "R " << static_cast<int>(color.R()) <<
    " G " << static_cast<int>(color.G()) <<
    " B " << static_cast<int>(color.B()) <<
    " A " << static_cast<int>(color.A());
    return out;
}

#endif //GRAPHICS_TINYRENDERER_COLOR_H
