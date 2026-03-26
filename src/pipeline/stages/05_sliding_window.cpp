/**
 * Etapa 05: sliding window, ajuste polinomial grado 2 y centro de carril.
 * Modelo fijo: x = a*y^2 + b*y + c.
 */

LaneState LanePipelineNode::run_sliding_window(const cv::Mat& bev_binary, cv::Mat& viz) {
    const int n_windows = 9;
    const int win_half_w = 30;
    const int min_pixels = 40;

    // Histograma de mitad inferior
    cv::Mat bottom_half = bev_binary(cv::Rect(0, bev_h_ / 2, bev_w_, bev_h_ / 2));
    cv::Mat hist;
    cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);

    // Picos L/R
    const int mid = bev_w_ / 2;
    cv::Point left_peak_loc;
    cv::Point right_peak_loc;
    double max_val = 0.0;

    cv::minMaxLoc(hist(cv::Rect(0, 0, mid, 1)), nullptr, &max_val, nullptr, &left_peak_loc);
    cv::minMaxLoc(hist(cv::Rect(mid, 0, mid, 1)), nullptr, &max_val, nullptr, &right_peak_loc);

    int cur_left_x = left_peak_loc.x;
    int cur_right_x = right_peak_loc.x + mid;

    const int win_h = bev_h_ / n_windows;
    std::vector<cv::Point2f> left_pts;
    std::vector<cv::Point2f> right_pts;

    for (int win = 0; win < n_windows; ++win) {
        const int y_low = bev_h_ - (win + 1) * win_h;

        auto extract_window = [&](int cx, std::vector<cv::Point2f>& pts, const cv::Scalar& color) {
            const int x_low = std::max(0, cx - win_half_w);
            const int x_high = std::min(bev_w_, cx + win_half_w);

            cv::Rect roi(x_low, y_low, x_high - x_low, win_h);
            cv::rectangle(viz, roi, color, 1);

            cv::Mat window = bev_binary(roi);
            std::vector<cv::Point> nz;
            cv::findNonZero(window, nz);

            if (static_cast<int>(nz.size()) >= min_pixels) {
                double sum_x = 0.0;
                for (const auto& p : nz) {
                    sum_x += p.x + x_low;
                }
                cx = static_cast<int>(sum_x / static_cast<double>(nz.size()));

                for (const auto& p : nz) {
                    pts.emplace_back(static_cast<float>(p.x + x_low),
                                     static_cast<float>(p.y + y_low));
                }
            }

            return cx;
        };

        cur_left_x = extract_window(cur_left_x, left_pts, cv::Scalar(255, 100, 0));
        cur_right_x = extract_window(cur_right_x, right_pts, cv::Scalar(0, 100, 255));
    }

    LaneState state;
    state.left = fit_polynomial(left_pts, bev_h_);
    state.right = fit_polynomial(right_pts, bev_h_);

    if (state.left.valid && state.right.valid) {
        state.center.a = (state.left.a + state.right.a) / 2.0;
        state.center.b = (state.left.b + state.right.b) / 2.0;
        state.center.c = (state.left.c + state.right.c) / 2.0;
        state.center.valid = true;
    } else if (state.left.valid) {
        const double lane_px = lane_width_m_ / bev_scale_mpp_;
        state.center.a = state.left.a;
        state.center.b = state.left.b;
        state.center.c = state.left.c + lane_px / 2.0;
        state.center.valid = true;
    } else if (state.right.valid) {
        const double lane_px = lane_width_m_ / bev_scale_mpp_;
        state.center.a = state.right.a;
        state.center.b = state.right.b;
        state.center.c = state.right.c - lane_px / 2.0;
        state.center.valid = true;
    }

    draw_polynomial(viz, state.left, bev_h_, cv::Scalar(255, 200, 0));
    draw_polynomial(viz, state.right, bev_h_, cv::Scalar(0, 200, 255));
    draw_polynomial(viz, state.center, bev_h_, cv::Scalar(0, 255, 0));

    return state;
}

PolyCoeffs LanePipelineNode::fit_polynomial(const std::vector<cv::Point2f>& pts, int img_height) {
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

void LanePipelineNode::draw_polynomial(cv::Mat& viz, const PolyCoeffs& c, int height, cv::Scalar color) {
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
