#ifndef GRAPHICS_TINYRENDERER_RENDERER_H
#define GRAPHICS_TINYRENDERER_RENDERER_H

#include "tgaimage.h"
#include "geometry.h"

// Bresenham's line algorithm
void draw_line(Vec2 p0, Vec2 p1, TGAImage &image, const Color &color);

// line sweeping triangle drawing
void draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, TGAImage &image, const Color &color);

// get barycentric
Vec3 get_barycentric(const Vec2 *pts, const Vec2& P);

//{
//    Vec3 x{pts[1].x - pts[0].x, pts[2].x - pts[0].x, pts[0].x - P.x};
//    Vec3 y{pts[1].y - pts[0].y, pts[2].y - pts[0].y, pts[0].y - P.y};
//    Vec3 u = cross(x, y);
//
//    // u.z is Int, so "abs(u.z) < 1" means "u.z == 0"
//    // if u.z is 0, then triangle is degenerate
//    if (std::abs(u.z) < 1) return {-1, 1, 1};
//
//    return {1.0f - (float)(u.x + u.y) / (float)u.z,
//            (float)u.x / (float)u.z,
//            (float)u.y / (float)u.z};
//}

//bool inside_triangle_cross_product(Vec2i *pts, const Vec2i& P) {
//    Vec2i AB(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
//    Vec2i BC(pts[2].x - pts[1].x, pts[2].y - pts[1].y);
//    Vec2i CA(pts[0].x - pts[2].x, pts[0].y - pts[2].y);
//
//    Vec2i AP(P.x - pts[0].x, P.y - pts[0].y);
//    Vec2i BP(P.x - pts[1].x, P.y - pts[1].y);
//    Vec2i CP(P.x - pts[2].x, P.y - pts[2].y);
//
//    int z1 = AB ^ AP;
//    int z2 = BC ^ BP;
//    int z3 = CA ^ CP;
//
//    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
//}
//
//// triangle drawing with barycentric
//void triangle_barycentric(Vec2i *pts, TGAImage &image, const Color& color) {
//    // create bounding box
//    Vec2i bbox_min(image.width() - 1, image.height() - 1);
//    Vec2i bbox_max(0, 0);
//    Vec2i clamp_min(0, 0);
//    Vec2i clamp_max(image.width() - 1, image.height() - 1);
//    for (int i = 0; i < 3; ++i) {
//        bbox_min.x = std::max(clamp_min.x, std::min(bbox_min.x, pts[i].x));
//        bbox_min.y = std::max(clamp_min.y, std::min(bbox_min.y, pts[i].y));
//
//        bbox_max.x = std::min(clamp_max.x, std::max(bbox_max.x, pts[i].x));
//        bbox_max.y = std::min(clamp_max.y, std::max(bbox_max.y, pts[i].y));
//    }
//
//    Vec2i P;
//    for (P.x = bbox_min.x; P.x <= bbox_max.x; ++P.x) {
//        for (P.y = bbox_min.y; P.y <= bbox_max.y ; ++P.y) {
//            Vec3f bc = barycentric(pts, P);
//            if (bc.x < 0 || bc.y < 0 || bc.z < 0) continue;
//            image.set_pixel(P.x, P.y, color);
//        }
//    }
//}
//
//// triangle drawing with cross product
//void triangle_cross_product(Vec2i *pts, TGAImage &image, const Color& color) {
//    // create bounding box
//    Vec2i bbox_min(image.width() - 1, image.height() - 1);
//    Vec2i bbox_max(0, 0);
//    Vec2i clamp_min(0, 0);
//    Vec2i clamp_max(image.width() - 1, image.height() - 1);
//    for (int i = 0; i < 3; ++i) {
//        bbox_min.x = std::max(clamp_min.x, std::min(bbox_min.x, pts[i].x));
//        bbox_min.y = std::max(clamp_min.y, std::min(bbox_min.y, pts[i].y));
//
//        bbox_max.x = std::min(clamp_max.x, std::max(bbox_max.x, pts[i].x));
//        bbox_max.y = std::min(clamp_max.y, std::max(bbox_max.y, pts[i].y));
//    }
//
//    Vec2i P;
//    for (P.x = bbox_min.x; P.x <= bbox_max.x; ++P.x) {
//        for (P.y = bbox_min.y; P.y <= bbox_max.y ; ++P.y) {
//            if (!inside_triangle_cross_product(pts, P)) continue;
//            image.set_pixel(P.x, P.y, color);
//        }
//    }
//}

#endif //GRAPHICS_TINYRENDERER_RENDERER_H
