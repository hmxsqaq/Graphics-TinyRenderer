#ifndef GRAPHICS_TINYRENDERER_COLOR_H
#define GRAPHICS_TINYRENDERER_COLOR_H

#include <cstdint>
#include <iostream>
#include <cassert>

struct Color {
    std::array<std::uint8_t, 4> bgra = {0, 0, 0, 0}; // BLUE, GREEN, RED, alpha
    std::uint8_t bytespp = 4; // number of bytes per pixel, 3 for RGB, 4 for RGBA
    std::uint8_t& operator[](const int i) { assert(i >= 0 && i < 4); return bgra[i]; }
    std::uint8_t R() const { return bgra[2]; }
    std::uint8_t G() const { return bgra[1]; }
    std::uint8_t B() const { return bgra[0]; }
    std::uint8_t A() const { return bgra[3]; }

    friend std::ostream& operator<<(std::ostream &out, const Color &color);
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
