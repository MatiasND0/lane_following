/**
 * Etapa 07: cálculo y publicación de e2, e3, k para MPC.
 * Convención fija del proyecto:
 * - origen en eje trasero
 * - Y hacia adelante
 * - X hacia la izquierda
 */

void LanePipelineNode::publish_lane_errors(const LaneState& state) {
    if (!state.center.valid) {
        return;
    }

    // Punto de evaluación en eje trasero (detrás de la cámara en BEV).
    const double y_eval_px = -camera_offset_m_ / bev_scale_mpp_;

    // e2 [m]
    const double x_center_px = state.center.a * y_eval_px * y_eval_px
                             + state.center.b * y_eval_px
                             + state.center.c;
    const double x_lane_center_m = (x_center_px - static_cast<double>(bev_w_) / 2.0) * bev_scale_mpp_;
    const double e2 = -x_lane_center_m;

    // e3 [rad]
    const double dxdy = 2.0 * state.center.a * y_eval_px + state.center.b;
    const double e3 = std::atan(dxdy);

    // k [m^-1]
    const double k_px_inv = 2.0 * state.center.a / std::pow(1.0 + dxdy * dxdy, 1.5);
    const double k = k_px_inv / bev_scale_mpp_;

    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
        static_cast<float>(e2),
        static_cast<float>(e3),
        static_cast<float>(k)
    };
    pub_errors_->publish(msg);

    RCLCPP_DEBUG(get_logger(), "e2=%.4f m  e3=%.4f rad  k=%.4f m^-1", e2, e3, k);
}
