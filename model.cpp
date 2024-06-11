#include <iostream>
#include <sstream>
#include "model.h"

Model::Model(const std::string& filename) {
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line);
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3 vert;
            for (int i = 0; i < 3; i++) iss >> vert[i];
            verts_.push_back(vert);
        } else if (!line.compare(0, 3, "vt ")) {
            iss >> trash >> trash;
            Vec2 uv;
            for (int i = 0; i < 2; i++) iss >> uv[i];
            tex_coord_.push_back({uv.x, 1 - uv.y});
        } else if (!line.compare(0, 3, "vn ")) {
            iss >> trash >> trash;
            Vec3 norm;
            for (int i = 0; i < 3; i++) iss >> norm[i];
            norms_.push_back(norm.normalize());
        }  else if (!line.compare(0, 2, "f ")) {
            int f, t, n;
            iss >> trash;
            int counter = 0;
            while (iss >> f >> trash >> t >> trash >> n) {
                facet_vrt_.push_back(--f);
                facet_tex_.push_back(--t);
                facet_nrm_.push_back(--n);
                counter++;
            }
            if (counter != 3) {
                std::cerr << "Error: the model file is supposed to be triangulated" << std::endl;
                return;
            }
        }
    }
    std::cout << "# v# " << n_verts() << " f# "  << n_faces() << " vt# " << tex_coord_.size() << " vn# " << norms_.size() << std::endl;
    diffuse_map_ = load_texture(filename, "_diffuse.tga");
    normal_map_ = load_texture(filename, "_nm_tangent.tga");
    specular_map_ = load_texture(filename, "_spec.tga");
}

Renderer Model::load_texture(const std::string& filename, const std::string& suffix) {
    size_t dot = filename.find_last_of('.');
    if (dot == std::string::npos) return {};
    std::string texture_file_name = filename.substr(0, dot) + suffix;
    Renderer renderer = TGAHandler::read_tga_file(texture_file_name);
    if (renderer.width() <= 0 || renderer.height() <= 0) return {};
    std::cout << "loading texture file: " << texture_file_name << std::endl;
    return renderer;
}
