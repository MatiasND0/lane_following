#include "lane_detection/lane_tracker.hpp"

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
    filter_coeff(prev_.center, raw.center, alpha);
}

}  // namespace lane_detection
