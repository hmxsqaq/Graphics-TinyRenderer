#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_() {
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line);
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f vert;
            for (float & i : vert.raw)
                iss >> i;
            verts_.push_back(vert);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<int> face;
            int itrash, index;
            iss >> trash;
            while (iss >> index >> trash >> itrash >> trash >> itrash) {
                index--;
                face.push_back(index);
            }
            faces_.push_back(face);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << std::endl;
}

Model::~Model() { }

Vec3f Model::vert(int idx) {
    return verts_[idx];
}

std::vector<int> Model::face(int idx) {
    return faces_[idx];
}

int Model::n_verts() const {
    return static_cast<int>(verts_.size());
}

int Model::n_faces() const {
    return static_cast<int>(faces_.size());
}
