#ifndef GRAPHICS_TINYRENDERER_RENDERER_H
#define GRAPHICS_TINYRENDERER_RENDERER_H

#include "tgaimage.h"
#include "geometry.h"

// Bresenham's line algorithm
void draw_line(Vec2 p0, Vec2 p1, TGAImage &image, const Color &color);

// line sweeping triangle drawing
void draw_triangle_linesweeping(Vec2 p0, Vec2 p1, Vec2 p2, TGAImage &image, const Color &color);

// get barycentric
Vec3 get_barycentric(const Vec2 *t, const Vec2& P);

// judge by cross product (like GAMES101)
bool is_inside_triangle_cross_product(Vec2 *t, const Vec2& P);

// triangle drawing with barycentric
void draw_triangle_barycentric(const Vec2 *t, TGAImage &image, const Color& color);


#endif //GRAPHICS_TINYRENDERER_RENDERER_H
