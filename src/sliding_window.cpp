#include "lane_detection/sliding_window.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

/**
 * sliding_window.cpp
 * -------------------
 * Detección de líneas por sliding window + ajuste polinomial grado 2.
 */

namespace lane_detection {

// --------------------------------------------------------------------------
// Funciones internas (declaration-only en este TU)
// --------------------------------------------------------------------------

static PolyCoeffs fit_polynomial(const std::vector<cv::Point2f>& pts, int img_height);
static void draw_polynomial(cv::Mat& viz, const PolyCoeffs& c, int height, cv::Scalar color);

// --------------------------------------------------------------------------
// detect_lanes — punto de entrada principal
// --------------------------------------------------------------------------

LaneState detect_lanes(const cv::Mat& bev_binary, cv::Mat& viz,
                       int bev_w, int bev_h,
                       const SlidingWindowParams& p) {
    const int win_h = bev_h / p.n_windows;

    // Histograma de mitad inferior
    cv::Mat bottom_half = bev_binary(cv::Rect(0, bev_h / 2, bev_w, bev_h / 2));
    cv::Mat hist;
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);

    // Picos L/R
    const int mid = bev_w / 2;
    cv::Point left_peak_loc;
    cv::Point right_peak_loc;
    double max_val = 0.0;

    cv::minMaxLoc(hist(cv::Rect(0, 0, mid, 1)),
                  nullptr, &max_val, nullptr, &left_peak_loc);
    cv::minMaxLoc(hist(cv::Rect(mid, 0, mid, 1)),
                  nullptr, &max_val, nullptr, &right_peak_loc);

    int cur_left_x  = left_peak_loc.x;
    int cur_right_x = right_peak_loc.x + mid;

    std::vector<cv::Point2f> left_pts;
    std::vector<cv::Point2f> right_pts;

    for (int win = 0; win < p.n_windows; ++win) {
        const int y_low = bev_h - (win + 1) * win_h;

        auto extract_window = [&](int cx, std::vector<cv::Point2f>& pts,
                                  const cv::Scalar& color) {
            const int x_low  = std::max(0, cx - p.win_half_w);
            const int x_high = std::min(bev_w, cx + p.win_half_w);

            cv::Rect roi(x_low, y_low, x_high - x_low, win_h);
            cv::rectangle(viz, roi, color, 1);

            cv::Mat window = bev_binary(roi);
            std::vector<cv::Point> nz;
            cv::findNonZero(window, nz);

            if (static_cast<int>(nz.size()) >= p.min_pixels) {
                double sum_x = 0.0;
                for (const auto& pt : nz) {
                    sum_x += pt.x + x_low;
                }
                cx = static_cast<int>(sum_x / static_cast<double>(nz.size()));

                for (const auto& pt : nz) {
                    pts.emplace_back(static_cast<float>(pt.x + x_low),
                                     static_cast<float>(pt.y + y_low));
                }
            }

            return cx;
        };

        cur_left_x  = extract_window(cur_left_x, left_pts, cv::Scalar(255, 100, 0));
        cur_right_x = extract_window(cur_right_x, right_pts, cv::Scalar(0, 100, 255));
    }

    LaneState state;
    state.left  = fit_polynomial(left_pts, bev_h);
    state.right = fit_polynomial(right_pts, bev_h);

    // --- Validación de ancho del carril ---
    if (state.left.valid && state.right.valid) {
        // Evaluar c_R - c_L en y = bev_h (base de la imagen)
        const double y_base = static_cast<double>(bev_h);
        const double u_L = state.left.a  * y_base * y_base + state.left.b  * y_base + state.left.c;
        const double u_R = state.right.a * y_base * y_base + state.right.b * y_base + state.right.c;
        const double ancho_m = (u_R - u_L) * p.bev_scale_mpp;

        if (ancho_m < 0.25 || ancho_m > 0.45) {
            state.width_warning = true;
        }
    }

    // --- Centro del carril ---
    const double lane_px = p.lane_width_m / p.bev_scale_mpp;

    if (state.left.valid && state.right.valid) {
        state.center.a = (state.left.a + state.right.a) / 2.0;
        state.center.b = (state.left.b + state.right.b) / 2.0;
        state.center.c = (state.left.c + state.right.c) / 2.0;
        state.center.valid = true;
    } else if (state.left.valid) {
        state.center.a = state.left.a;
        state.center.b = state.left.b;
        state.center.c = state.left.c + lane_px / 2.0;
        state.center.valid = true;
    } else if (state.right.valid) {
        state.center.a = state.right.a;
        state.center.b = state.right.b;
        state.center.c = state.right.c - lane_px / 2.0;
        state.center.valid = true;
    }

    // --- Visualización ---
    draw_polynomial(viz, state.left,   bev_h, cv::Scalar(255, 200, 0));
    draw_polynomial(viz, state.right,  bev_h, cv::Scalar(0, 200, 255));
    draw_polynomial(viz, state.center, bev_h, cv::Scalar(0, 255, 0));

    return state;
}

// --------------------------------------------------------------------------
// fit_polynomial — ajuste por mínimos cuadrados (SVD)
// --------------------------------------------------------------------------

static PolyCoeffs fit_polynomial(const std::vector<cv::Point2f>& pts, int img_height) {
    PolyCoeffs coeffs;
    if (static_cast<int>(pts.size()) < 20) {
        return coeffs;
    }

    const int n = static_cast<int>(pts.size());
    cv::Mat A(n, 3, CV_64F);
    cv::Mat B(n, 1, CV_64F);

    const double y_scale = static_cast<double>(img_height);
    for (int i = 0; i < n; ++i) {
        const double y = pts[i].y / y_scale;
        A.at<double>(i, 0) = y * y;
        A.at<double>(i, 1) = y;
        A.at<double>(i, 2) = 1.0;
        B.at<double>(i, 0) = pts[i].x;
    }

    cv::Mat result;
    const bool ok = cv::solve(A, B, result, cv::DECOMP_SVD);
    if (!ok) {
        return coeffs;
    }

    coeffs.a = result.at<double>(0, 0) / (y_scale * y_scale);
    coeffs.b = result.at<double>(1, 0) / y_scale;
    coeffs.c = result.at<double>(2, 0);
    coeffs.valid = true;

    return coeffs;
}

// --------------------------------------------------------------------------
// draw_polynomial — dibuja el polinomio sobre la visualización
// --------------------------------------------------------------------------

static void draw_polynomial(cv::Mat& viz, const PolyCoeffs& c,
                            int height, cv::Scalar color) {
    if (!c.valid) {
        return;
    }

    std::vector<cv::Point> pts;
    for (int y = 0; y < height; y += 2) {
        const int x = static_cast<int>(c.a * y * y + c.b * y + c.c);
        if (x >= 0 && x < viz.cols) {
            pts.emplace_back(x, y);
        }
    }

    if (pts.size() > 1) {
        cv::polylines(viz, pts, false, color, 2);
    }
}

}  // namespace lane_detection
