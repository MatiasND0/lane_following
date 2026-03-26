#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "lane_detection/types.hpp"

/**
 * sliding_window.hpp
 * -------------------
 * Detección de líneas por sliding window + ajuste polinomial grado 2.
 * Modelo fijo: x = a·y² + b·y + c.
 */

namespace lane_detection {

struct SlidingWindowParams {
    int n_windows    = 9;
    int win_half_w   = 30;       // px
    int min_pixels   = 40;
    double lane_width_m  = 0.35;
    double bev_scale_mpp = 0.005;
};

/// Ejecuta sliding window sobre BEV binarizado.
/// Dibuja las ventanas y polinomios sobre viz.
/// Valida ancho del carril y setea width_warning en LaneState.
LaneState detect_lanes(const cv::Mat& bev_binary, cv::Mat& viz,
                       int bev_w, int bev_h,
                       const SlidingWindowParams& params);

}  // namespace lane_detection
