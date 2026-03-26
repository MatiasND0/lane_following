#include "lane_detection/error_calculator.hpp"

#include <cmath>

/**
 * error_calculator.cpp
 * ---------------------
 * Cálculo de e2, e3, k para el controlador MPC.
 * Convención fija: origen en eje trasero, Y adelante, X izquierda.
 * Fórmulas exactas de COORDINATES.md.
 */

namespace lane_detection {

LaneErrors compute_lane_errors(const LaneState& filtered, const BevConfig& cfg) {
    LaneErrors errors;

    if (!filtered.center.valid) {
        return errors;
    }

    // Punto de evaluación en eje trasero (detrás de la cámara en BEV).
    const double y_eval_px = -(cfg.camera_longitudinal_offset_m / cfg.bev_scale_mpp);

    // e2 [m]: error lateral
    const double x_center_px = filtered.center.a * y_eval_px * y_eval_px
                             + filtered.center.b * y_eval_px
                             + filtered.center.c;
    errors.e2 = -(x_center_px - static_cast<double>(cfg.bev_w) / 2.0) * cfg.bev_scale_mpp;

    // e3 [rad]: error angular
    const double dxdy = 2.0 * filtered.center.a * y_eval_px + filtered.center.b;
    errors.e3 = std::atan(dxdy);

    // k [m⁻¹]: curvatura
    const double k_px_inv = 2.0 * filtered.center.a
                          / std::pow(1.0 + dxdy * dxdy, 1.5);
    errors.k = k_px_inv / cfg.bev_scale_mpp;

    errors.valid = true;
    return errors;
}

}  // namespace lane_detection
