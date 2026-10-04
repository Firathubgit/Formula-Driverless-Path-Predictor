#pragma once
#include <QColor>
#include <cmath>

// One vertex of the scene's drawn geometry: a position and a linear colour with its alpha, as the scene's materials read
// them. Colours are given as displayed (sRGB) and stored linear; the materials encode them again for display.
struct SceneVertex { float x, y, z, r, g, b, a; };

inline SceneVertex scene_vertex(double x, double y, double z, const QColor& c, double alpha = 1) {
    const auto linear = [](float v) { return v <= 0.04045f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f); };
    return {float(x), float(y), float(z), linear(c.redF()), linear(c.greenF()), linear(c.blueF()), float(alpha)};
}
