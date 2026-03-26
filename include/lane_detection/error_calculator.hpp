#pragma once

#include "lane_detection/types.hpp"

/**
 * error_calculator.hpp
 * ---------------------
 * Cálculo de e2, e3, k para el controlador MPC.
 * Convención fija: origen en eje trasero, Y adelante, X izquierda.
 * Fórmulas exactas de COORDINATES.md.
 */

namespace lane_detection {

LaneErrors compute_lane_errors(const LaneState& filtered, const BevConfig& cfg);

}  // namespace lane_detection
