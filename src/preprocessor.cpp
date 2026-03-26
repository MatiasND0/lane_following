#include "lane_detection/preprocessor.hpp"

#include <opencv2/imgproc.hpp>

/**
 * preprocessor.cpp
 * ----------------
 * Preprocesamiento HLS: máscara binaria de líneas blancas y amarillas.
 */

namespace lane_detection {

cv::Mat compute_mask(const cv::Mat& bgr, const HlsParams& p) {
    cv::Mat hls;
    cv::GaussianBlur(bgr, hls, cv::Size(5, 5), 0);
    cv::cvtColor(hls, hls, cv::COLOR_BGR2HLS);

    // Líneas blancas: L alto, S bajo
    cv::Mat white_mask;
    cv::inRange(hls,
        cv::Scalar(0, p.white_l_min, 0),
        cv::Scalar(180, p.white_l_max, p.white_s_max),
        white_mask);

    // Líneas amarillas: H amarillo, S alto
    cv::Mat yellow_mask;
    cv::inRange(hls,
        cv::Scalar(p.yellow_h_min, p.yellow_l_min, p.yellow_s_min),
        cv::Scalar(p.yellow_h_max, 220, 255),
        yellow_mask);

    // Fusión
    cv::Mat combined;
    cv::bitwise_or(white_mask, yellow_mask, combined);

    // Cierre morfológico para continuidad de línea
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, kernel);

    return combined;
}

}  // namespace lane_detection
