#include "lane_detection/lane_tracker.hpp"

#include <algorithm>
#include <cmath>

/**
 * lane_tracker.cpp
 * -----------------
 * Filtrado temporal exponencial de coeficientes polinomiales.
 * Rechaza saltos espurios de curvatura.
 * Si width_warning == true: usa alpha/2 (más conservador).
 */

namespace lane_detection {

LaneTracker::LaneTracker(double alpha) : alpha_(alpha) {}

void LaneTracker::update(const LaneState& raw) {
    // Seleccionar alpha: si el ancho es sospechoso, filtrar más suave
    const double alpha = raw.width_warning ? (alpha_ / 2.0) : alpha_;
    double alpha_center = alpha;

    // Si MHT cambia de hipótesis con salto grande en C (px), amortiguar más.
    if (prev_.center.valid && raw.center.valid) {
        const double jump_px = std::abs(raw.center.c - prev_.center.c);
        const double jump_ref_px = 10.0;
        if (jump_px > jump_ref_px) {
            const double scale = std::clamp(jump_ref_px / jump_px, 0.35, 1.0);
            alpha_center *= scale;
        }
    }
    alpha_center = std::clamp(alpha_center, 0.08, 1.0);

    auto filter_coeff = [](PolyCoeffs& filtered, const PolyCoeffs& raw_c, double a) {
        if (!raw_c.valid) {
            return;
        }

        if (!filtered.valid) {
            filtered = raw_c;
            return;
        }

        filtered.a = a * raw_c.a + (1.0 - a) * filtered.a;
        filtered.b = a * raw_c.b + (1.0 - a) * filtered.b;
        filtered.c = a * raw_c.c + (1.0 - a) * filtered.c;
    };

    filter_coeff(prev_.left, raw.left, alpha);
    filter_coeff(prev_.right, raw.right, alpha);
    filter_coeff(prev_.center, raw.center, alpha_center);
}

}  // namespace lane_detection
