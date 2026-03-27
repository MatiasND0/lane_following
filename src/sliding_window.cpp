#include "lane_detection/sliding_window.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>

/**
 * sliding_window.cpp
 * -------------------
 * Detección de líneas por sliding window + ajuste polinomial grado 2.
 * Mejorado con inferencia de carril, validación de Ego Center y filtrado por inercia/mediana.
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
    const double lane_px = p.lane_width_m / p.bev_scale_mpp;
    const int min_separation_px = static_cast<int>(0.5 * lane_px);

    // Variables estáticas para mantener memoria del cuadro anterior si perdemos ambas líneas
    static int last_left_x = (bev_w / 2) - static_cast<int>(lane_px / 2.0);
    static int last_right_x = (bev_w / 2) + static_cast<int>(lane_px / 2.0);

    // Histograma de mitad inferior
    cv::Mat bottom_half = bev_binary(cv::Rect(0, bev_h / 2, bev_w, bev_h / 2));
    cv::Mat hist;
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);

    const int mid = bev_w / 2;
    cv::Point left_peak_loc, right_peak_loc;
    double max_val_l = 0.0, max_val_r = 0.0;

    // --- MEJORA 1: BÚSQUEDA LOCALIZADA DEL HISTOGRAMA ---
    const int search_margin = p.win_half_w * 2;

    // Límites para la izquierda
    int l_min = std::max(0, last_left_x - search_margin);
    int l_max = std::min(mid - 1, last_left_x + search_margin);
    if (l_max > l_min) {
        cv::minMaxLoc(hist(cv::Rect(l_min, 0, l_max - l_min, 1)), nullptr, &max_val_l, nullptr, &left_peak_loc);
        left_peak_loc.x += l_min; 
    }

    // Límites para la derecha
    int r_min = std::max(mid, last_right_x - search_margin);
    int r_max = std::min(bev_w - 1, last_right_x + search_margin);
    if (r_max > r_min) {
        cv::minMaxLoc(hist(cv::Rect(r_min, 0, r_max - r_min, 1)), nullptr, &max_val_r, nullptr, &right_peak_loc);
        right_peak_loc.x += r_min; 
    }

    int cur_left_x  = left_peak_loc.x;
    int cur_right_x = right_peak_loc.x;

    // --- LÓGICA DE ROBUSTEZ: Validación desde el Ego Center ---
    const double intensity_threshold = p.min_pixels * 3.0; 
    
    bool left_valid = max_val_l > intensity_threshold;
    bool right_valid = max_val_r > intensity_threshold;

    const int ego_center = bev_w / 2;
    const double expected_dist = lane_px / 2.0;
    const double tolerance = expected_dist * 0.4; // 40% de tolerancia

    // 1. Validar la línea IZQUIERDA respecto al auto
    if (left_valid) {
        double dist_to_center = ego_center - cur_left_x;
        if (dist_to_center < 0 || dist_to_center > (expected_dist + tolerance)) {
            left_valid = false;
        }
    }

    // 2. Validar la línea DERECHA respecto al auto
    if (right_valid) {
        double dist_to_center = cur_right_x - ego_center;
        if (dist_to_center < 0 || dist_to_center > (expected_dist + tolerance)) {
            right_valid = false;
        }
    }

    // 3. Inferir la posición faltante
    if (left_valid && !right_valid) {
        cur_right_x = cur_left_x + static_cast<int>(lane_px);
    } else if (!left_valid && right_valid) {
        cur_left_x = cur_right_x - static_cast<int>(lane_px);
    } else if (!left_valid && !right_valid) {
        cur_left_x = last_left_x;
        cur_right_x = last_right_x;
    }

    last_left_x = cur_left_x;
    last_right_x = cur_right_x;
    // --- FIN LÓGICA DE ROBUSTEZ ---

    std::vector<cv::Point2f> left_pts;
    std::vector<cv::Point2f> right_pts;

    for (int win = 0; win < p.n_windows; ++win) {
        const int y_low = bev_h - (win + 1) * win_h;

        auto extract_window = [&](int cx, std::vector<cv::Point2f>& pts,
                      const cv::Scalar& color,
                      bool exclude_near_left) {
            const int x_low  = std::max(0, cx - p.win_half_w);
            const int x_high = std::min(bev_w, cx + p.win_half_w);

            cv::Rect roi(x_low, y_low, x_high - x_low, win_h);
            cv::rectangle(viz, roi, color, 1);

            cv::Mat window = bev_binary(roi);
            std::vector<cv::Point> nz;
            cv::findNonZero(window, nz);

            std::vector<cv::Point2f> accepted_pts;
            std::vector<float> x_coords; 
            accepted_pts.reserve(nz.size());
            x_coords.reserve(nz.size());
            
            for (const auto& pt : nz) {
                const int global_x = pt.x + x_low;
                if (exclude_near_left && std::abs(global_x - cur_left_x) < min_separation_px) {
                    continue;
                }
                accepted_pts.emplace_back(static_cast<float>(global_x), static_cast<float>(pt.y + y_low));
                x_coords.push_back(static_cast<float>(global_x));
            }

            const int window_energy = static_cast<int>(accepted_pts.size());

            if (window_energy >= p.min_pixels) {
                // --- MEJORA 2: USO DE MEDIANA EN LUGAR DE PROMEDIO ---
                std::sort(x_coords.begin(), x_coords.end());
                int new_cx = static_cast<int>(x_coords[x_coords.size() / 2]);

                // Inercia
                const int max_shift_px = static_cast<int>(p.win_half_w * 0.4); 
                int delta_x = new_cx - cx;

                if (std::abs(delta_x) > max_shift_px) {
                    cx += (delta_x > 0) ? max_shift_px : -max_shift_px;
                } else {
                    cx = new_cx;
                }

                if (window_energy >= static_cast<int>(p.min_pixels * 1.2)) {
                    pts.insert(pts.end(), accepted_pts.begin(), accepted_pts.end());
                }
            }

            return cx;
        };

        cur_left_x  = extract_window(cur_left_x, left_pts, cv::Scalar(255, 100, 0), false);
        cur_right_x = extract_window(cur_right_x, right_pts, cv::Scalar(0, 100, 255), true);
    }

    LaneState state;
    state.left  = fit_polynomial(left_pts, bev_h);
    state.right = fit_polynomial(right_pts, bev_h);

    // --- Validación de ancho del carril ---
    if (state.left.valid && state.right.valid) {
        const double y_base = static_cast<double>(bev_h);
        const double u_L = state.left.a  * y_base * y_base + state.left.b  * y_base + state.left.c;
        const double u_R = state.right.a * y_base * y_base + state.right.b * y_base + state.right.c;
        const double ancho_m = (u_R - u_L) * p.bev_scale_mpp;

        if (ancho_m < 0.25 || ancho_m > 0.45) {
            state.width_warning = true;
        }
    }

    // --- Centro del carril ---
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

    // --- Kalman sobre centro (a, b, c) ---
    static cv::KalmanFilter kf;
    static bool kf_configured = false;
    static bool kf_initialized = false;

    if (!kf_configured) {
        kf.init(3, 3, 0, CV_32F);
        kf.transitionMatrix = cv::Mat::eye(3, 3, CV_32F);
        kf.measurementMatrix = cv::Mat::eye(3, 3, CV_32F);
        kf.processNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 1e-4f;
        kf.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 1e-2f;
        kf.errorCovPost = cv::Mat::eye(3, 3, CV_32F);
        kf_configured = true;
    }

    cv::Mat prediction;
    if (kf_initialized) {
        prediction = kf.predict();
    }

    if (state.center.valid) {
        cv::Mat measurement(3, 1, CV_32F);
        measurement.at<float>(0, 0) = static_cast<float>(state.center.a);
        measurement.at<float>(1, 0) = static_cast<float>(state.center.b);
        measurement.at<float>(2, 0) = static_cast<float>(state.center.c);

        if (!kf_initialized) {
            kf.statePost = measurement.clone();
            kf.statePre = measurement.clone();
            kf_initialized = true;
        } else {
            const cv::Mat estimated = kf.correct(measurement);
            state.center.a = static_cast<double>(estimated.at<float>(0, 0));
            state.center.b = static_cast<double>(estimated.at<float>(1, 0));
            state.center.c = static_cast<double>(estimated.at<float>(2, 0));
            state.center.valid = true;
        }
    } else if (kf_initialized) {
        state.center.a = static_cast<double>(prediction.at<float>(0, 0));
        state.center.b = static_cast<double>(prediction.at<float>(1, 0));
        state.center.c = static_cast<double>(prediction.at<float>(2, 0));
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