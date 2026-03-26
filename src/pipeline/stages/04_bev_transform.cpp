/**
 * Etapa 04: configuración de homografía BEV y proyección inversa para overlay.
 *
 * Los puntos src/dst son parámetros ROS2 dinámicos que pueden ajustarse
 * desde la GUI de calibración sin recompilar.
 */

void LanePipelineNode::init_bev_homography() {
    // Leer puntos desde parámetros ROS2 (ya declarados en 01_params_and_init)
    const auto src_vec = get_parameter("bev_src_pts").as_double_array();
    const auto dst_vec = get_parameter("bev_dst_pts").as_double_array();

    if (src_vec.size() != 8 || dst_vec.size() != 8) {
        RCLCPP_ERROR(get_logger(),
            "bev_src_pts y bev_dst_pts deben tener 8 elementos cada uno (4 pares x,y).");
        return;
    }

    const std::vector<cv::Point2f> src_pts = {
        {static_cast<float>(src_vec[0]), static_cast<float>(src_vec[1])},
        {static_cast<float>(src_vec[2]), static_cast<float>(src_vec[3])},
        {static_cast<float>(src_vec[4]), static_cast<float>(src_vec[5])},
        {static_cast<float>(src_vec[6]), static_cast<float>(src_vec[7])},
    };

    const std::vector<cv::Point2f> dst_pts = {
        {static_cast<float>(dst_vec[0]), static_cast<float>(dst_vec[1])},
        {static_cast<float>(dst_vec[2]), static_cast<float>(dst_vec[3])},
        {static_cast<float>(dst_vec[4]), static_cast<float>(dst_vec[5])},
        {static_cast<float>(dst_vec[6]), static_cast<float>(dst_vec[7])},
    };

    H_ = cv::getPerspectiveTransform(src_pts, dst_pts);
    H_inv_ = cv::getPerspectiveTransform(dst_pts, src_pts);

    RCLCPP_INFO(get_logger(),
        "Homografía BEV actualizada. src=[%.0f,%.0f | %.0f,%.0f | %.0f,%.0f | %.0f,%.0f]",
        src_vec[0], src_vec[1], src_vec[2], src_vec[3],
        src_vec[4], src_vec[5], src_vec[6], src_vec[7]);
}

void LanePipelineNode::draw_lane_overlay(cv::Mat& bgr, const LaneState& state) {
    if (!state.center.valid) {
        return;
    }

    std::vector<cv::Point2f> bev_pts;
    for (int y = 0; y < bev_h_; y += 5) {
        const double x = state.center.a * y * y + state.center.b * y + state.center.c;
        if (x >= 0.0 && x < static_cast<double>(bev_w_)) {
            bev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
    }
    if (bev_pts.empty()) {
        return;
    }

    std::vector<cv::Point2f> orig_pts;
    cv::perspectiveTransform(bev_pts, orig_pts, H_inv_);

    std::vector<cv::Point> poly_pts;
    poly_pts.reserve(orig_pts.size());
    for (const auto& p : orig_pts) {
        poly_pts.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
    }

    cv::polylines(bgr, poly_pts, false, cv::Scalar(0, 255, 0), 3);
}
