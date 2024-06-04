#ifndef GRAPHICS_TINYRENDERER_TGAIMAGE_H
#define GRAPHICS_TINYRENDERER_TGAIMAGE_H

#include <cstdint>
#include <fstream>
#include <vector>

// standard TGA header
#pragma pack(push, 1)
struct TGAHeader {
    std::uint8_t  id_length = 0;
    std::uint8_t  color_map_type = 0;
    std::uint8_t  data_type_code = 0;
    std::uint16_t color_map_origin = 0;
    std::uint16_t color_map_length = 0;
    std::uint8_t  color_map_depth = 0;
    std::uint16_t x_origin = 0;
    std::uint16_t y_origin = 0;
    std::uint16_t width = 0;
    std::uint16_t height = 0;
    std::uint8_t  bits_per_pixel = 0;
    std::uint8_t  image_descriptor = 0;
};
#pragma pack(pop)

struct Color {
    std::uint8_t bgra[4] = {0,0,0,0}; // blue, green, red, alpha
    std::uint8_t bytespp = 4; // number of bytes per pixel, 3 for RGB, 4 for RGBA
    std::uint8_t& operator[](const int i) { return bgra[i]; }
};

std::ostream& operator<<(std::ostream& out, const Color& color);

class TGAImage {
public:
    // TGAImage color formats
    enum Format { GRAYSCALE = 1, RGB = 3, RGBA = 4 };

    TGAImage() = default;
    TGAImage(int width, int height, int bits_per_pixel);

    bool read_tga_file(const std::string& filename);
    bool write_tga_file(const std::string& filename, bool v_flip = true, bool rle = true) const;

    void flip_horizontally();
    void flip_vertically();

    Color get_pixel(int x, int y) const;
    void set_pixel(int x, int y, const Color &color);

    int width() const { return width_; };
    int height() const { return height_;};
private:
    bool load_rle_data(std::ifstream &in);
    bool unload_rle_data(std::ofstream &out) const;

    int width_ = 0;
    int height_ = 0;
    std::uint8_t bpp_ = 0; // bits per pixel
    std::vector<std::uint8_t> data_ = {};
};

#endif //GRAPHICS_TINYRENDERER_TGAIMAGE_H
