#pragma once

#include <opencv2/core.hpp>

#include "lane_detection/types.hpp"

/**
 * preprocessor.hpp
 * ----------------
 * Preprocesamiento HLS: genera máscara binaria de líneas blancas y amarillas.
 */

namespace lane_detection {

struct HlsParams {
    int white_l_min = 160, white_l_max = 255, white_s_max = 140;
    int yellow_h_min = 15, yellow_h_max = 35;
    int yellow_l_min = 80, yellow_l_max = 220;
    int yellow_s_min = 100, yellow_s_max = 255;
};

cv::Mat compute_mask(const cv::Mat& bgr, const HlsParams& params);

}  // namespace lane_detection
