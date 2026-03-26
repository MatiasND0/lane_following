#include "lane_detection/bev_transformer.hpp"

#include <opencv2/imgproc.hpp>

/**
 * bev_transformer.cpp
 * --------------------
 * Homografía BEV.
 * Modo "points": 4 pares src/dst.
 * Modo "model": TODO.
 */

namespace lane_detection {

// --------------------------------------------------------------------------
// Constructor
// --------------------------------------------------------------------------

BevTransformer::BevTransformer(const BevConfig& cfg)
    : bev_w_(cfg.bev_w), bev_h_(cfg.bev_h) {
    compute_from_points(cfg);
}

// --------------------------------------------------------------------------
// API pública
// --------------------------------------------------------------------------

void BevTransformer::reconfigure(const BevConfig& cfg) {
    bev_w_ = cfg.bev_w;
    bev_h_ = cfg.bev_h;

    // TODO: if (cfg.mode == "model") { compute_from_model(cfg); return; }
    compute_from_points(cfg);
}

cv::Mat BevTransformer::transform(const cv::Mat& binary) const {
    cv::Mat output;
    cv::warpPerspective(binary, output, H_, cv::Size(bev_w_, bev_h_));
    return output;
}

cv::Mat BevTransformer::project_to_image(const LaneState& state,
                                         const cv::Size& img_size) const {
    cv::Mat overlay = cv::Mat::zeros(img_size, CV_8UC3);

    if (!state.center.valid || H_inv_.empty()) {
        return overlay;
    }

    std::vector<cv::Point2f> bev_pts;
    for (int y = 0; y < bev_h_; y += 5) {
        const double x = state.center.a * y * y
                       + state.center.b * y
                       + state.center.c;
        if (x >= 0.0 && x < static_cast<double>(bev_w_)) {
            bev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
    }
    if (bev_pts.empty()) {
        return overlay;
    }

    std::vector<cv::Point2f> orig_pts;
    cv::perspectiveTransform(bev_pts, orig_pts, H_inv_);

    std::vector<cv::Point> poly_pts;
    poly_pts.reserve(orig_pts.size());
    for (const auto& p : orig_pts) {
        poly_pts.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
    }

    cv::polylines(overlay, poly_pts, false, cv::Scalar(0, 255, 0), 3);
    return overlay;
}

// --------------------------------------------------------------------------
// Modo "points": 4 puntos src → dst
// --------------------------------------------------------------------------

void BevTransformer::compute_from_points(const BevConfig& cfg) {
    if (cfg.src_pts.size() != 8 || cfg.dst_pts.size() != 8) {
        return;
    }

    const std::vector<cv::Point2f> src = {
        {static_cast<float>(cfg.src_pts[0]), static_cast<float>(cfg.src_pts[1])},
        {static_cast<float>(cfg.src_pts[2]), static_cast<float>(cfg.src_pts[3])},
        {static_cast<float>(cfg.src_pts[4]), static_cast<float>(cfg.src_pts[5])},
        {static_cast<float>(cfg.src_pts[6]), static_cast<float>(cfg.src_pts[7])},
    };

    const std::vector<cv::Point2f> dst = {
        {static_cast<float>(cfg.dst_pts[0]), static_cast<float>(cfg.dst_pts[1])},
        {static_cast<float>(cfg.dst_pts[2]), static_cast<float>(cfg.dst_pts[3])},
        {static_cast<float>(cfg.dst_pts[4]), static_cast<float>(cfg.dst_pts[5])},
        {static_cast<float>(cfg.dst_pts[6]), static_cast<float>(cfg.dst_pts[7])},
    };

    H_     = cv::getPerspectiveTransform(src, dst);
    H_inv_ = cv::getPerspectiveTransform(dst, src);
}

}  // namespace lane_detection
