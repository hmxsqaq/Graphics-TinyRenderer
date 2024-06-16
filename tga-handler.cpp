#include "tga-handler.h"

Texture TGAHandler::read_tga_file(const std::string &filename) {
    // open file
    std::ifstream in;
    in.open(filename, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "read - cannot open file: " << filename << "\n";
        return {};
    }

    // read header
    TGAHeader header;
    in.read(reinterpret_cast<char *>(&header), sizeof(header));
    if (!in.good()) {
        std::cerr << "read - cannot read the TGA header\n";
        return {};
    }

    // load width & height & bpp from header
    auto width = header.width;
    auto height = header.height;
    auto bpp = header.bits_per_pixel >> 3; // /= 8
    if (width <= 0 || height <= 0 || (bpp != Color::GRAYSCALE &&
                                        bpp != Color::RGB &&
                                        bpp != Color::RGBA)) {
        std::cerr << "read - dad width/height/bpp value: width " << width << " height " << height << " bpp " << bpp << "\n";
        return {};
    }

    Texture texture(width, height, bpp);

    // read frame_data
    if (header.data_type_code == 3 || header.data_type_code == 2) {
        in.read(reinterpret_cast<char *>(texture.data.data()), (long long)width * height * bpp);
        if (!in.good()) {
            std::cerr << "read - cannot read frame data\n";
            return {};
        }
    } else if (header.data_type_code == 10 || header.data_type_code == 11) {
        if (!load_rle_data(in, texture)) {
            std::cerr << "read - cannot load RLE data\n";
            return {};
        }
    } else {
        std::cerr << "read - unknown file format " << (int)header.data_type_code << "\n";
        return {};
    }

    // flip or not
    if (!(header.image_descriptor & 0x20))
        texture.flip_vertically();
    if (header.image_descriptor & 0x10)
        texture.flip_horizontally();
    return texture;
}

bool TGAHandler::write_tga_file(const std::string &filename,
                                const int width,
                                const int height,
                                const std::uint8_t bpp,
                                const unsigned char *data,
                                bool v_flip, bool rle) {
    constexpr std::uint8_t developer_area_ref[4] = {0, 0, 0, 0};
    constexpr std::uint8_t extension_area_ref[4] = {0, 0, 0, 0};
    constexpr std::uint8_t footer[18] = {'T','R','U','E','V','I','S','I','O','N','-','X','F','I','L','E','.','\0'};

    // open file
    std::ofstream out;
    out.open(filename, std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "write - cannot open file: " << filename << "\n";
        return false;
    }

    std::cout << "write - width " << width << " height " << height << " bpp " << (int)bpp << "\n";

    // prepare header
    TGAHeader header = {};
    header.bits_per_pixel = bpp << 3; // = * 8
    header.width  = width;
    header.height = height;
    header.data_type_code = (bpp == Color::GRAYSCALE ? (rle ? 11 : 3) : (rle ? 10 : 2));
    header.image_descriptor = v_flip ? 0x00 : 0x20; // top-left or bottom-left origin

    // write header
    out.write(reinterpret_cast<const char *>(&header), sizeof(header));
    if (!out.good()) {
        std::cerr << "write - cannot dump the tga file\n";
        return false;
    }

    // write img
    if (!rle) {
        out.write(reinterpret_cast<const char *>(data), width * height * bpp);
        if (!out.good()) {
            std::cerr << "write - cannot unload raw frame_data\n";
            return false;
        }
    } else if (!unload_rle_data(out, width, height, bpp, data)) {
        std::cerr << "write - cannot unload rle frame_data\n";
        return false;
    }
    out.write(reinterpret_cast<const char *>(developer_area_ref), sizeof(developer_area_ref));
    if (!out.good()) {
        std::cerr << "write - cannot dump the tga file\n";
        return false;
    }
    out.write(reinterpret_cast<const char *>(extension_area_ref), sizeof(extension_area_ref));
    if (!out.good()) {
        std::cerr << "write - cannot dump the tga file\n";
        return false;
    }

    // write footer
    out.write(reinterpret_cast<const char *>(footer), sizeof(footer));
    if (!out.good()) {
        std::cerr << "write - cannot dump the tga file\n";
        return false;
    }
    return true;
}


bool TGAHandler::load_rle_data(std::ifstream &in, Texture &texture) {
    auto width = texture.width;
    auto height = texture.height;
    auto bpp = texture.bpp;

    size_t pixel_count = width * height;
    size_t current_pixel = 0;
    size_t current_byte  = 0;
    Color color_buffer;
    do {
        std::uint8_t chunk_header;
        chunk_header = in.get();
        if (!in.good()) {
            std::cerr << "rle - an error occurred while reading the frame_data\n";
            return false;
        }
        if (chunk_header < 128) {
            chunk_header++;
            for (int i = 0; i < chunk_header; i++) {
                in.read(reinterpret_cast<char *>(color_buffer.bgra.data()), bpp);
                if (!in.good()) {
                    std::cerr << "rle - an error occurred while reading the header\n";
                    return false;
                }
                for (int t = 0; t < bpp; t++)
                    texture.data[current_byte++] = color_buffer.bgra[t];
                current_pixel++;
                if (current_pixel > pixel_count) {
                    std::cerr << "rle - too many pixels were read\n";
                    return false;
                }
            }
        } else {
            chunk_header -= 127;
            in.read(reinterpret_cast<char *>(color_buffer.bgra.data()), bpp);
            if (!in.good()) {
                std::cerr << "rle - an error occurred while reading the header\n";
                return false;
            }
            for (int i=0; i < chunk_header; i++) {
                for (int t = 0; t < bpp; t++)
                    texture.data[current_byte++] = color_buffer.bgra[t];
                current_pixel++;
                if (current_pixel > pixel_count) {
                    std::cerr << "rle - too many pixels were read\n";
                    return false;
                }
            }
        }
    } while (current_pixel < pixel_count);
    return true;
}

// Run-Length Encoding: compress the output
// AAAAABBBCCDAA -> 5A3B2C1D2A
bool TGAHandler::unload_rle_data(std::ofstream &out,
                                 const int width,
                                 const int height,
                                 const std::uint8_t bpp,
                                 const unsigned char *data) {
    const std::uint8_t max_chunk_length = 128;
    size_t n_pixels = width * height;
    size_t current_pixel = 0;
    while (current_pixel < n_pixels) {
        size_t chunk_start = current_pixel * bpp;
        size_t current_byte = current_pixel * bpp;
        std::uint8_t run_length = 1;
        bool raw = true;
        while (current_pixel + run_length < n_pixels && run_length < max_chunk_length) {
            bool is_equal = true;
            for (int t = 0; is_equal && t < bpp; t++)
                is_equal = (data[current_byte + t] == data[current_byte + t + bpp]);
            current_byte += bpp;
            if (run_length == 1)
                raw = !is_equal;
            if (raw && is_equal) {
                run_length--;
                break;
            }
            if (!raw && !is_equal)
                break;
            run_length++;
        }
        current_pixel += run_length;
        out.put(static_cast<char>(raw ? run_length - 1 : run_length + 127));
        if (!out.good()) {
            std::cerr << "rle - can't dump the tga file\n";
            return false;
        }
        out.write(reinterpret_cast<const char *>(data + chunk_start), (raw ? run_length * bpp : bpp));
        if (!out.good()) {
            std::cerr << "rle - can't dump the tga file\n";
            return false;
        }
    }
    return true;
}