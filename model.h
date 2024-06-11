#ifndef GRAPHICS_TINYRENDERER_MODEL_H
#define GRAPHICS_TINYRENDERER_MODEL_H

#include <vector>
#include <string>
#include "geometry.h"
#include "tgahandler.h"

class Model {
public:
    explicit Model(const std::string& filename);

    const Renderer& diffuse()  const { return diffuse_map_;  }
    const Renderer& specular() const { return specular_map_; }
    int n_verts() const { return static_cast<int>(verts_.size()); }
    int n_faces() const { return static_cast<int>(facet_vrt_.size() / 3); }
    Vec3 vert(const int i)                          const { return verts_[i]; }
    Vec3 vert(const int i_face, const int i_vert)   const { return verts_[facet_vrt_[i_face * 3 + i_vert]]; }
    Vec2 uv(const int i_face, const int i_vert)     const { return tex_coord_[facet_tex_[i_face * 3 + i_vert]]; }
    Vec3 normal(const int i_face, const int i_vert) const { return norms_[facet_nrm_[i_face * 3 + i_vert]]; }
    Vec3 normal(const Vec2 &uvf)                    const {
        Color c = normal_map_.get_pixel((int)uvf[0] * normal_map_.width(), (int )uvf[1] * normal_map_.height());
        return Vec3{(double)c[2], (double)c[1], (double)c[0]}*2./255. - Vec3{1,1,1};
    }
private:
    std::vector<Vec3> verts_{};     // vertices
    std::vector<Vec2> tex_coord_{}; // tex coords
    std::vector<Vec3> norms_{};     // normal
    std::vector<int> facet_vrt_{};  // facet vertices index
    std::vector<int> facet_tex_{};  // facet texture index
    std::vector<int> facet_nrm_{};  // facet normal index
    Renderer diffuse_map_{};        // diffuse color texture
    Renderer normal_map_{};         // normal map texture
    Renderer specular_map_{};       // specular map texture
    static Renderer load_texture(const std::string& filename, const std::string& suffix);
};


#endif //GRAPHICS_TINYRENDERER_MODEL_H
