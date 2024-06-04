#include <iostream>
#include "tgaimage.h"

TGAImage::TGAImage(const int width, const int height, const int bits_per_pixel)
    : width_(width), height_(height), bpp_(bits_per_pixel), data_(width_ * height_ * bpp_, 0)
    { }

bool TGAImage::read_tga_file(const std::string& filename) {
    // open file
    std::ifstream in;
    in.open(filename, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "TGAImage::read_tga_file: cannot open file " << filename << "\n";
        return false;
    }

    // read header
    TGAHeader header;
    in.read(reinterpret_cast<char *>(&header), sizeof(header));
    if (!in.good()) {
        std::cerr << "TGAImage::read_tga_file: cannot read the header\n";
        return false;
    }

    // load width & height & bpp from header
    width_ = header.width;
    height_ = header.height;
    bpp_ = header.bits_per_pixel;
    if (width_ <= 0 || height_ <= 0 || (bpp_ != GRAYSCALE &&
                                        bpp_ != RGB &&
                                        bpp_ != RGBA)) {
        std::cerr << "TGAImage::read_tga_file: bad width/height/bpp value\n";
        return false;
    }

    // allocate memory
    size_t n_bytes = width_ * height_ * bpp_;
    data_ = std::vector<std::uint8_t>(n_bytes, 0);

    // read data
    if (header.data_type_code == 3 || header.data_type_code == 2) {
        in.read(reinterpret_cast<char *>(data_.data()), (long long)n_bytes);
        if (!in.good()) {
            std::cerr << "TGAImage::read_tga_file: cannot read the data\n";
            return false;
        }
    } else if (header.data_type_code == 10 || header.data_type_code == 11) {
        if (!load_rle_data(in)) {
            std::cerr << "TGAImage::read_tga_file: cannot read the data\n";
            return false;
        }
    } else {
        std::cerr << "TGAImage::read_tga_file: unknown file format " << (int)header.data_type_code << "\n";
        return false;
    }

    // flip or not
    if (!(header.image_descriptor & 0x20))
        flip_vertically();
    if (header.image_descriptor & 0x10)
        flip_horizontally();
    std::cerr << width_ << " x " << height_ << " / " << bpp_ * 8 << "\n";
    return true;
}

bool TGAImage::write_tga_file(const std::string& filename, const bool v_flip, const bool rle) const {
    constexpr std::uint8_t developer_area_ref[4] = {0, 0, 0, 0};
    constexpr std::uint8_t extension_area_ref[4] = {0, 0, 0, 0};
    constexpr std::uint8_t footer[18] = {'T','R','U','E','V','I','S','I','O','N','-','X','F','I','L','E','.','\0'};

    // open file
    std::ofstream out;
    out.open(filename, std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "TGAImage::write_tga_file: cannot open file " << filename << "\n";
        return false;
    }

    // prepare header
    TGAHeader header = {};
    header.bits_per_pixel = bpp_ << 3; // = * 8
    header.width  = width_;
    header.height = height_;
    header.data_type_code = (bpp_ == GRAYSCALE ? (rle ? 11 : 3) : (rle ? 10 : 2));
    header.image_descriptor = v_flip ? 0x00 : 0x20; // top-left or bottom-left origin

    // write header
    out.write(reinterpret_cast<const char *>(&header), sizeof(header));
    if (!out.good()) {
        std::cerr << "TGAImage::write_tga_file: cannot dump the tga file\n";
        return false;
    }

    // write img
    if (!rle) {
        out.write(reinterpret_cast<const char *>(data_.data()), width_ * height_ * bpp_);
        if (!out.good()) {
            std::cerr << "TGAImage::write_tga_file: cannot unload raw data\n";
            return false;
        }
    } else if (!unload_rle_data(out)) {
        std::cerr << "TGAImage::write_tga_file: cannot unload rle data\n";
        return false;
    }
    out.write(reinterpret_cast<const char *>(developer_area_ref), sizeof(developer_area_ref));
    if (!out.good()) {
        std::cerr << "TGAImage::write_tga_file: cannot dump the tga file\n";
        return false;
    }
    out.write(reinterpret_cast<const char *>(extension_area_ref), sizeof(extension_area_ref));
    if (!out.good()) {
        std::cerr << "TGAImage::write_tga_file: cannot dump the tga file\n";
        return false;
    }

    // write footer
    out.write(reinterpret_cast<const char *>(footer), sizeof(footer));
    if (!out.good()) {
        std::cerr << "TGAImage::write_tga_file: cannot dump the tga file\n";
        return false;
    }
    return true;
}

void TGAImage::flip_horizontally() {
    int half = width_ >> 1;
    for (int x = 0; x < half; ++x) {
        for (int y = 0; y < height_; ++y) {
            for (int b = 0; b < bpp_; ++b) {
                std::swap(data_[(x + y * width_) * bpp_ + b],
                          data_[(width_ - 1 - x + y * width_) * bpp_ + b]);
            }
        }
    }
}

void TGAImage::flip_vertically() {
    int half = height_ >> 1;
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < half; ++y) {
            for (int b = 0; b < bpp_; ++b) {
                std::swap(data_[(x + y * width_) * bpp_ + b],
                          data_[(x + (height_ - 1 - y) * width_) * bpp_ + b]);
            }
        }
    }
}

Color TGAImage::get_pixel(const int x, const int y) const {
    if (data_.empty() || x < 0 || y < 0 || x >= width_ || y >= height_)
        return {};
    Color ret = {0, 0, 0, 0, bpp_};
    const std::uint8_t *pixel = data_.data() + (x + y * width_) * bpp_;
    for (int i = 0; i < bpp_; i++) ret.bgra[i] = pixel[i];
    return ret;
}

void TGAImage::set_pixel(const int x, const int y, const Color &color) {
    if (data_.empty() || x<0 || y<0 || x >= width_ || y >= height_) return;
    memcpy(data_.data() + (x + y * width_) * bpp_, color.bgra, bpp_);
}

bool TGAImage::load_rle_data(std::ifstream &in) {
    size_t pixel_count = width_ * height_;
    size_t current_pixel = 0;
    size_t current_byte  = 0;
    Color color_buffer;
    do {
        std::uint8_t chunk_header;
        chunk_header = in.get();
        if (!in.good()) {
            std::cerr << "TGAImage::load_rle_data: an error occurred while read the data\n";
            return false;
        }
        if (chunk_header < 128) {
            chunk_header++;
            for (int i = 0; i < chunk_header; i++) {
                in.read(reinterpret_cast<char *>(color_buffer.bgra), bpp_);
                if (!in.good()) {
                    std::cerr << "TGAImage::load_rle_data: an error occurred while reading the header\n";
                    return false;
                }
                for (int t = 0; t < bpp_; t++)
                    data_[current_byte++] = color_buffer.bgra[t];
                current_pixel++;
                if (current_pixel > pixel_count) {
                    std::cerr << "TGAImage::load_rle_data: Too many pixels read\n";
                    return false;
                }
            }
        } else {
            chunk_header -= 127;
            in.read(reinterpret_cast<char *>(color_buffer.bgra), bpp_);
            if (!in.good()) {
                std::cerr << "TGAImage::load_rle_data: an error occurred while reading the header\n";
                return false;
            }
            for (int i=0; i < chunk_header; i++) {
                for (int t = 0; t < bpp_; t++)
                    data_[current_byte++] = color_buffer.bgra[t];
                current_pixel++;
                if (current_pixel > pixel_count) {
                    std::cerr << "TGAImage::load_rle_data: Too many pixels read\n";
                    return false;
                }
            }
        }
    } while (current_pixel < pixel_count);
    return true;
}

// Run-Length Encoding: compress the output
// AAAAABBBCCDAA -> 5A3B2C1D2A
bool TGAImage::unload_rle_data(std::ofstream &out) const {
    const std::uint8_t max_chunk_length = 128;
    size_t n_pixels = width_ * height_;
    size_t current_pixel = 0;
    while (current_pixel < n_pixels) {
        size_t chunk_start = current_pixel * bpp_;
        size_t current_byte = current_pixel * bpp_;
        std::uint8_t run_length = 1;
        bool raw = true;
        while (current_pixel + run_length < n_pixels && run_length < max_chunk_length) {
            bool is_equal = true;
            for (int t = 0; is_equal && t < bpp_; t++)
                is_equal = (data_[current_byte + t] == data_[current_byte + t + bpp_]);
            current_byte += bpp_;
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
            std::cerr << "can't dump the tga file\n";
            return false;
        }
        out.write(reinterpret_cast<const char *>(data_.data() + chunk_start), (raw ? run_length * bpp_ : bpp_));
        if (!out.good()) {
            std::cerr << "can't dump the tga file\n";
            return false;
        }
    }
    return true;
}

std::ostream &operator<<(std::ostream &out, const Color &color) {
    for (unsigned char i : color.bgra) out << (int)i << " ";
    return out;
}
