#ifndef GRAPHICS_TINYRENDERER_TGA_HANDLER_H
#define GRAPHICS_TINYRENDERER_TGA_HANDLER_H

#include "renderer.h"
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

class TGAHandler {
public:
    TGAHandler() = delete;

    static Renderer read_tga_file(const std::string& filename);
    static bool write_tga_file(const std::string& filename,const Renderer& renderer, bool v_flip = true, bool rle = true);
private:
    static bool load_rle_data(std::ifstream &in, Renderer& renderer);
    static bool unload_rle_data(std::ofstream &out, const Renderer& renderer);
};

#endif //GRAPHICS_TINYRENDERER_TGA_HANDLER_H
