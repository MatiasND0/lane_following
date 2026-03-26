/**
 * Etapa 02: lectura secuencial de frames desde disco y orquestación del ciclo.
 */

void LanePipelineNode::load_frame_list() {
    for (const auto& entry : fs::directory_iterator(frames_dir_)) {
        const auto ext = entry.path().extension();
        if (ext == ".jpg" || ext == ".png") {
            frame_paths_.push_back(entry.path().string());
        }
    }
    std::sort(frame_paths_.begin(), frame_paths_.end());
}

void LanePipelineNode::process_next_frame() {
    // Recalcular homografía si los parámetros cambiaron desde la GUI
    if (bev_dirty_.exchange(false)) {
        init_bev_homography();
    }

    if (paused_ && !step_once_) {
        return;
    }

    if (step_once_) {
        step_once_ = false;
    }

    if (frame_idx_ >= static_cast<int>(frame_paths_.size())) {
        RCLCPP_INFO_ONCE(get_logger(), "Todos los frames procesados.");
        return;
    }

    cv::Mat frame = cv::imread(frame_paths_[frame_idx_++]);
    if (frame.empty()) {
        RCLCPP_WARN(get_logger(), "Frame vacío en idx %d, saltando.", frame_idx_ - 1);
        return;
    }

    // 1) Original
    publish_image(pub_original_, frame, "original");

    // 2) HLS
    cv::Mat hls_mask = compute_hls_mask(frame);
    cv::Mat hls_mask_viz;
    cv::cvtColor(hls_mask, hls_mask_viz, cv::COLOR_GRAY2BGR);
    publish_image(pub_hls_mask_, hls_mask_viz, "hls_mask");

    // 3) BEV
    cv::Mat bev_binary;
    cv::warpPerspective(hls_mask, bev_binary, H_, cv::Size(bev_w_, bev_h_));

    cv::Mat bev_viz;
    cv::cvtColor(bev_binary, bev_viz, cv::COLOR_GRAY2BGR);
    publish_image(pub_bev_, bev_viz, "bev");

    // 4) Sliding window + fit
    cv::Mat sliding_viz = bev_viz.clone();
    const LaneState raw_state = run_sliding_window(bev_binary, sliding_viz);
    publish_image(pub_sliding_, sliding_viz, "sliding_window");

    // 5) Filtro temporal
    apply_temporal_filter(raw_state);

    // 6) Overlay
    cv::Mat overlay = frame.clone();
    draw_lane_overlay(overlay, state_filtered_);
    publish_image(pub_overlay_, overlay, "overlay");

    // 7) Salida al MPC
    publish_lane_errors(state_filtered_);
}

void LanePipelineNode::on_control_cmd(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& cmd = msg->data;

    if (cmd == "pause") {
        paused_ = true;
        RCLCPP_INFO(get_logger(), "Control: pause");
        return;
    }

    if (cmd == "resume") {
        paused_ = false;
        RCLCPP_INFO(get_logger(), "Control: resume");
        return;
    }

    if (cmd == "toggle") {
        paused_ = !paused_;
        RCLCPP_INFO(get_logger(), "Control: toggle -> paused=%s", paused_ ? "true" : "false");
        return;
    }

    if (cmd == "step") {
        paused_ = true;
        step_once_ = true;
        RCLCPP_DEBUG(get_logger(), "Control: step");
        return;
    }

    if (cmd == "reset") {
        frame_idx_ = 0;
        state_filtered_ = LaneState{};
        RCLCPP_INFO(get_logger(), "Control: reset (frame_idx=0)");
        return;
    }

    RCLCPP_WARN(get_logger(), "Control desconocido en /lane_control/cmd: '%s'", cmd.c_str());
}
