#include "lane_detection/sliding_window.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

/**
 * sliding_window.cpp
 * -------------------
 * Detección de líneas por sliding window + ajuste polinomial grado 2.
 * Mejorado con inferencia, validación de Ego Center, filtrado por inercia/mediana,
 * y Multiple Hypothesis Tracking (MHT) depurado con similitud de forma.
 */

namespace lane_detection
{

    // --------------------------------------------------------------------------
    // Funciones internas (declaration-only en este TU)
    // --------------------------------------------------------------------------

    static PolyCoeffs fit_polynomial(const std::vector<cv::Point2f> &pts, int img_height);
    static void draw_polynomial(cv::Mat &viz, const PolyCoeffs &c, int height, cv::Scalar color, int thickness = 2);

    // --------------------------------------------------------------------------
    // detect_lanes — punto de entrada principal
    // --------------------------------------------------------------------------

    LaneState detect_lanes(const cv::Mat &bev_binary, cv::Mat &viz,
                           int bev_w, int bev_h,
                           const SlidingWindowParams &p)
    {
        const int win_h = bev_h / p.n_windows;
        const double lane_px = p.lane_width_m / p.bev_scale_mpp;
        const int min_separation_px = static_cast<int>(0.5 * lane_px);

        // Variables estáticas para mantener memoria de la posición inicial del histograma
        static int last_left_hist_x = (bev_w / 2) - static_cast<int>(lane_px / 2.0);
        static int last_right_hist_x = (bev_w / 2) + static_cast<int>(lane_px / 2.0);

        // Histograma de mitad inferior
        cv::Mat bottom_half = bev_binary(cv::Rect(0, bev_h / 2, bev_w, bev_h / 2));
        cv::Mat hist;
        cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);

        const int mid = bev_w / 2;
        cv::Point left_peak_loc, right_peak_loc;
        double max_val_l = 0.0, max_val_r = 0.0;

        // --- BÚSQUEDA LOCALIZADA DEL HISTOGRAMA (MEMORIA TEMPORAL DE INICIO) ---
        const int search_margin_hist = p.win_half_w * 3;

        int l_min = std::max(0, last_left_hist_x - search_margin_hist);
        int l_max = std::min(mid - 1, last_left_hist_x + search_margin_hist);
        if (l_max > l_min)
        {
            cv::minMaxLoc(hist(cv::Rect(l_min, 0, l_max - l_min, 1)), nullptr, &max_val_l, nullptr, &left_peak_loc);
            left_peak_loc.x += l_min;
        }
        else
        {
            left_peak_loc.x = last_left_hist_x;
        }

        int r_min = std::max(mid, last_right_hist_x - search_margin_hist);
        int r_max = std::min(bev_w - 1, last_right_hist_x + search_margin_hist);
        if (r_max > r_min)
        {
            cv::minMaxLoc(hist(cv::Rect(r_min, 0, r_max - r_min, 1)), nullptr, &max_val_r, nullptr, &right_peak_loc);
            right_peak_loc.x += r_min;
        }
        else
        {
            right_peak_loc.x = last_right_hist_x;
        }

        int cur_left_x = left_peak_loc.x;
        int cur_right_x = right_peak_loc.x;

        // --- LÓGICA DE ROBUSTEZ: Validación desde el Ego Center ---
        const double intensity_threshold = p.min_pixels * 3.0;
        bool left_valid_init = max_val_l > intensity_threshold;
        bool right_valid_init = max_val_r > intensity_threshold;

        const int ego_center = bev_w / 2;
        const double expected_dist = lane_px / 2.0;
        const double tolerance_init = expected_dist * 0.5;

        // Validar picos iniciales contra el auto
        if (left_valid_init)
        {
            double dist = ego_center - cur_left_x;
            if (dist < 0 || dist > (expected_dist + tolerance_init))
                left_valid_init = false;
        }
        if (right_valid_init)
        {
            double dist = cur_right_x - ego_center;
            if (dist < 0 || dist > (expected_dist + tolerance_init))
                right_valid_init = false;
        }

        // Inferencia geométrica inicial para la ventana deslizante
        if (left_valid_init && !right_valid_init)
            cur_right_x = cur_left_x + static_cast<int>(lane_px);
        else if (!left_valid_init && right_valid_init)
            cur_left_x = cur_right_x - static_cast<int>(lane_px);

        // Guardar para el próximo histograma
        const double alpha_hist = 0.1;
        last_left_hist_x = static_cast<int>((alpha_hist * cur_left_x) + ((1.0 - alpha_hist) * last_left_hist_x));
        last_right_hist_x = static_cast<int>((alpha_hist * cur_right_x) + ((1.0 - alpha_hist) * last_right_hist_x));
        // --- FIN LÓGICA DE ROBUSTEZ ---

        std::vector<cv::Point2f> left_pts;
        std::vector<cv::Point2f> right_pts;

        for (int win = 0; win < p.n_windows; ++win)
        {
            const int y_low = bev_h - (win + 1) * win_h;

            auto extract_window = [&](int &cx, std::vector<cv::Point2f> &pts,
                                      const cv::Scalar &color,
                                      bool exclude_near_left)
            {
                const int x_low = std::max(0, cx - p.win_half_w);
                const int x_high = std::min(bev_w, cx + p.win_half_w);

                cv::Rect roi(x_low, y_low, x_high - x_low, win_h);
                cv::rectangle(viz, roi, color, 1);

                cv::Mat window = bev_binary(roi);
                std::vector<cv::Point> nz;
                cv::findNonZero(window, nz);

                std::vector<float> x_coords;
                std::vector<cv::Point2f> temp_accepted_pts;

                temp_accepted_pts.reserve(nz.size());
                x_coords.reserve(nz.size());

                for (const auto &pt : nz)
                {
                    const int global_x = pt.x + x_low;
                    if (exclude_near_left && std::abs(global_x - cur_left_x) < min_separation_px)
                        continue;
                    temp_accepted_pts.emplace_back(static_cast<float>(global_x), static_cast<float>(pt.y + y_low));
                    x_coords.push_back(static_cast<float>(global_x));
                }

                if (static_cast<int>(temp_accepted_pts.size()) >= p.min_pixels)
                {
                    // USO DE MEDIANA
                    std::sort(x_coords.begin(), x_coords.end());
                    int new_cx = static_cast<int>(x_coords[x_coords.size() / 2]);

                    // Inercia lateral estricta
                    const int max_shift_px = static_cast<int>(p.win_half_w * 0.3);
                    int delta_x = new_cx - cx;
                    if (std::abs(delta_x) > max_shift_px)
                        cx += (delta_x > 0) ? max_shift_px : -max_shift_px;
                    else
                        cx = new_cx;

                    // Filtrado de energía
                    if (static_cast<int>(temp_accepted_pts.size()) >= static_cast<int>(p.min_pixels * 1.5))
                    {
                        pts.insert(pts.end(), temp_accepted_pts.begin(), temp_accepted_pts.end());
                    }
                }
                return cx;
            };

            cur_left_x = extract_window(cur_left_x, left_pts, cv::Scalar(255, 100, 0), false);
            cur_right_x = extract_window(cur_right_x, right_pts, cv::Scalar(0, 100, 255), true);
        }

        LaneState state;
        state.left = fit_polynomial(left_pts, bev_h);
        state.right = fit_polynomial(right_pts, bev_h);

        // ==========================================================================
        // MHT CON SEMÁNTICA DESACOPLADA Y VALIDACIÓN DE EGO-CENTER
        // ==========================================================================
        static PolyCoeffs last_best_center;
        static bool has_history = false;

        struct Hypothesis
        {
            PolyCoeffs poly;
            double error_pos;
            double error_shape;
            double error_ego; // NUEVO: Penalización por estar fuera del auto
            double total_error;
            bool valid;
            cv::Scalar color;
        };

        Hypothesis h_both = {{0, 0, 0, false}, 1e9, 1e9, 1e9, 1e9, false, cv::Scalar(255, 0, 255)};     // Fucsia
        Hypothesis h_L_is_L = {{0, 0, 0, false}, 1e9, 1e9, 1e9, 1e9, false, cv::Scalar(255, 255, 0)};   // Cian
        Hypothesis h_L_is_R = {{0, 0, 0, false}, 1e9, 1e9, 1e9, 1e9, false, cv::Scalar(255, 255, 255)}; // Blanco
        Hypothesis h_R_is_R = {{0, 0, 0, false}, 1e9, 1e9, 1e9, 1e9, false, cv::Scalar(0, 255, 255)};   // Amarillo
        Hypothesis h_R_is_L = {{0, 0, 0, false}, 1e9, 1e9, 1e9, 1e9, false, cv::Scalar(0, 0, 255)};     // Rojo

        if (state.left.valid && state.right.valid)
        {
            double dist = std::abs(state.right.c - state.left.c);
            if (dist > lane_px * 0.6 && dist < lane_px * 1.4)
            {
                h_both.poly.a = (state.left.a + state.right.a) / 2.0;
                h_both.poly.b = (state.left.b + state.right.b) / 2.0;
                h_both.poly.c = (state.left.c + state.right.c) / 2.0;
                h_both.poly.valid = true;
                h_both.valid = true;
            }
        }

        if (state.left.valid)
        {
            const double center_shift_px = (lane_px / 2.0) + p.center_inference_extra_px;

            h_L_is_L.poly = state.left;
            h_L_is_L.poly.c += center_shift_px;
            h_L_is_L.poly.valid = true;
            h_L_is_L.valid = true;

            h_L_is_R.poly = state.left;
            h_L_is_R.poly.c -= center_shift_px;
            h_L_is_R.poly.valid = true;
            h_L_is_R.valid = true;
        }

        if (state.right.valid)
        {
            const double center_shift_px = (lane_px / 2.0) + p.center_inference_extra_px;

            h_R_is_R.poly = state.right;
            h_R_is_R.poly.c -= center_shift_px;
            h_R_is_R.poly.valid = true;
            h_R_is_R.valid = true;

            h_R_is_L.poly = state.right;
            h_R_is_L.poly.c += center_shift_px;
            h_R_is_L.poly.valid = true;
            h_R_is_L.valid = true;
        }

        state.center.valid = false;

        if (!has_history)
        {
            double best_c_dist = 1e9;
            std::vector<Hypothesis *> init_hyps = {&h_both, &h_L_is_L, &h_L_is_R, &h_R_is_R, &h_R_is_L};
            for (auto h : init_hyps)
            {
                if (h->valid)
                {
                    double d = std::abs(h->poly.c - bev_w / 2.0);
                    if (d < best_c_dist)
                    {
                        best_c_dist = d;
                        state.center = h->poly;
                    }
                }
            }
            if (state.center.valid)
            {
                last_best_center = state.center;
                has_history = true;
            }
        }
        else
        {
            auto calc_pos_error = [&](const PolyCoeffs &cand) -> double
            {
                if (!cand.valid)
                    return 1e9;
                double err = 0;
                std::vector<std::pair<double, double>> y_evals = {{static_cast<double>(bev_h), 1.0}, {bev_h * 0.6, 0.5}};
                for (auto &y_w : y_evals)
                {
                    double x_cand = cand.a * y_w.first * y_w.first + cand.b * y_w.first + cand.c;
                    double x_hist = last_best_center.a * y_w.first * y_w.first + last_best_center.b * y_w.first + last_best_center.c;
                    err += std::abs(x_cand - x_hist) * y_w.second;
                }
                return err / 1.5;
            };

            auto calc_shape_error = [&](const PolyCoeffs &cand) -> double
            {
                if (!cand.valid)
                    return 1e9;
                double diff_a = std::abs(cand.a - last_best_center.a) * 1e6;
                double diff_b = std::abs(cand.b - last_best_center.b) * 10.0;
                return diff_a + diff_b;
            };

            // --- NUEVA MÉTRICA: ERROR DE EGO-CENTER ---
            auto calc_ego_error = [&](const PolyCoeffs &cand) -> double
            {
                if (!cand.valid)
                    return 1e9;
                // Evaluamos la X del polinomio en la parte inferior de la imagen (donde está el auto)
                double x_bottom = cand.a * bev_h * bev_h + cand.b * bev_h + cand.c;
                double dist_from_ego = std::abs(x_bottom - (bev_w / 2.0));

                // Límite: 70% del ancho del carril. Si está más lejos, es el carril vecino.
                if (dist_from_ego > lane_px * 0.70)
                {
                    return 10000.0; // Penalización mortal (Muro de ladrillos)
                }

                // Penalidad suave para desempatar a favor de la más centrada si la historia es dudosa
                return dist_from_ego * 0.1;
            };

            std::vector<Hypothesis *> hyps = {&h_both, &h_L_is_L, &h_L_is_R, &h_R_is_R, &h_R_is_L};

            if (h_both.valid)
                h_both.total_error -= 10.0;

            for (auto h : hyps)
            {
                if (h->valid)
                {
                    h->error_pos = calc_pos_error(h->poly);
                    h->error_shape = calc_shape_error(h->poly);
                    h->error_ego = calc_ego_error(h->poly); // Agregamos la evaluación

                    // Sumamos el error_ego a la cuenta total
                    h->total_error = h->error_pos + (h->error_shape * 25.0) + h->error_ego;
                }
            }

            Hypothesis *best_h = nullptr;
            double min_err = 1e9;
            for (auto h : hyps)
            {
                if (h->valid && h->total_error < min_err)
                {
                    min_err = h->total_error;
                    best_h = h;
                }
            }

            const double max_pos_jump = lane_px * 0.8;

            if (best_h && best_h->error_pos < max_pos_jump)
            {
                const double jump_px = std::abs(best_h->poly.c - last_best_center.c);
                const double jump_ref_px = lane_px * 0.15;
                const double scale = std::clamp(jump_ref_px / std::max(jump_px, 1e-6), 0.35, 1.0);
                const double alpha_smooth = std::clamp(0.30 * scale, 0.12, 0.40);
                last_best_center.a = (alpha_smooth * best_h->poly.a) + ((1.0 - alpha_smooth) * last_best_center.a);
                last_best_center.b = (alpha_smooth * best_h->poly.b) + ((1.0 - alpha_smooth) * last_best_center.b);
                last_best_center.c = (alpha_smooth * best_h->poly.c) + ((1.0 - alpha_smooth) * last_best_center.c);
            }

            // Publicar siempre el centro suavizado para evitar saltos discretos
            // cuando MHT cambia de hipótesis frame a frame.
            state.center = last_best_center;
        }

        // ==========================================================================
        // VISUALIZACIÓN DE DEPURACIÓN
        // ==========================================================================
        draw_polynomial(viz, state.left, bev_h, cv::Scalar(255, 200, 0), 2);
        draw_polynomial(viz, state.right, bev_h, cv::Scalar(0, 100, 255), 2);

        std::vector<Hypothesis *> all_hyps = {&h_both, &h_L_is_L, &h_L_is_R, &h_R_is_R, &h_R_is_L};
        for (auto h : all_hyps)
        {
            if (h->valid)
                draw_polynomial(viz, h->poly, bev_h, h->color, 1);
        }

        draw_polynomial(viz, state.center, bev_h, cv::Scalar(0, 255, 0), 3);

        return state;
    }

    // --------------------------------------------------------------------------
    // fit_polynomial — ajuste por mínimos cuadrados (SVD)
    // --------------------------------------------------------------------------

    static PolyCoeffs fit_polynomial(const std::vector<cv::Point2f> &pts, int img_height)
    {
        PolyCoeffs coeffs;
        if (static_cast<int>(pts.size()) < 15)
        {
            return coeffs;
        }

        const int n = static_cast<int>(pts.size());
        cv::Mat A(n, 3, CV_64F);
        cv::Mat B(n, 1, CV_64F);

        const double y_scale = static_cast<double>(img_height);
        for (int i = 0; i < n; ++i)
        {
            const double y = pts[i].y / y_scale;
            A.at<double>(i, 0) = y * y;
            A.at<double>(i, 1) = y;
            A.at<double>(i, 2) = 1.0;
            B.at<double>(i, 0) = pts[i].x;
        }

        cv::Mat result;
        const bool ok = cv::solve(A, B, result, cv::DECOMP_SVD);
        if (!ok)
        {
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

    static void draw_polynomial(cv::Mat &viz, const PolyCoeffs &c,
                                int height, cv::Scalar color, int thickness)
    {
        if (!c.valid)
            return;

        std::vector<cv::Point> pts;
        for (int y = 0; y < height; y += 4)
        {
            const int x = static_cast<int>(c.a * y * y + c.b * y + c.c);
            if (x >= 0 && x < viz.cols)
                pts.emplace_back(x, y);
        }

        if (pts.size() > 1)
            cv::polylines(viz, pts, false, color, thickness);
    }

} // namespace lane_detection
