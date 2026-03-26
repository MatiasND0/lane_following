/**
 * Etapa 01: parámetros, publishers, timer e inicialización general del nodo.
 */

LanePipelineNode::LanePipelineNode() : Node("lane_pipeline_node") {
    // --- Parámetros ROS2 ---
    declare_parameter("frames_dir", std::string(""));
    declare_parameter("publish_rate_hz", 25.0);
    declare_parameter("bev_scale_mpp", 0.005);
    declare_parameter("alpha_filter", 0.3);
    declare_parameter("lane_width_m", 0.35);
    declare_parameter("camera_offset_m", 0.23);
    declare_parameter("start_paused", false);

    // --- Puntos de homografía BEV (4 pares x,y = 8 doubles) ---
    // Defaults: trapecio proporcional sobre imagen 640x480
    //   src: top-left, top-right, bottom-right, bottom-left
    declare_parameter("bev_src_pts", std::vector<double>{
        640.0 * 0.40, 480.0 * 0.55,   // top-left
        640.0 * 0.60, 480.0 * 0.55,   // top-right
        640.0 * 0.95, 480.0 * 0.95,   // bottom-right
        640.0 * 0.05, 480.0 * 0.95,   // bottom-left
    });
    //   dst: rectángulo centrado en BEV 320x240
    declare_parameter("bev_dst_pts", std::vector<double>{
        320.0 * 0.10,   0.0,           // top-left
        320.0 * 0.90,   0.0,           // top-right
        320.0 * 0.90, 240.0,           // bottom-right
        320.0 * 0.10, 240.0,           // bottom-left
    });

    frames_dir_ = get_parameter("frames_dir").as_string();
    bev_scale_mpp_ = get_parameter("bev_scale_mpp").as_double();
    alpha_filter_ = get_parameter("alpha_filter").as_double();
    lane_width_m_ = get_parameter("lane_width_m").as_double();
    camera_offset_m_ = get_parameter("camera_offset_m").as_double();
    paused_ = get_parameter("start_paused").as_bool();
    const double rate_hz = get_parameter("publish_rate_hz").as_double();

    if (frames_dir_.empty()) {
        RCLCPP_ERROR(get_logger(), "Parámetro 'frames_dir' no seteado. Uso:");
        RCLCPP_ERROR(get_logger(), "  ros2 run lane_detection lane_pipeline_node --ros-args -p frames_dir:=/path/to/frames");
        rclcpp::shutdown();
        return;
    }

    // --- Fuente offline ---
    load_frame_list();
    if (frame_paths_.empty()) {
        RCLCPP_ERROR(get_logger(), "No se encontraron imágenes .jpg/.png en: %s", frames_dir_.c_str());
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(get_logger(), "Cargados %zu frames desde %s", frame_paths_.size(), frames_dir_.c_str());

    // --- Publishers de debug ---
    const auto qos = rclcpp::QoS(10);
    pub_original_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/original", qos);
    pub_hls_mask_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/hls_mask", qos);
    pub_bev_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/bev", qos);
    pub_sliding_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/sliding_window", qos);
    pub_overlay_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/overlay", qos);

    // --- Publisher de salida MPC ---
    pub_errors_ = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_errors_est", qos);

    // --- Control de reproducción (GUI/CLI) ---
    sub_control_cmd_ = create_subscription<std_msgs::msg::String>(
        "/lane_control/cmd",
        qos,
        std::bind(&LanePipelineNode::on_control_cmd, this, std::placeholders::_1));

    // --- Inicialización BEV ---
    init_bev_homography();

    // --- Callback de reconfiguración dinámica de parámetros ---
    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
            -> rcl_interfaces::msg::SetParametersResult
        {
            for (const auto& p : params) {
                if (p.get_name() == "bev_src_pts" || p.get_name() == "bev_dst_pts") {
                    bev_dirty_.store(true);
                    RCLCPP_INFO(get_logger(), "Parámetros BEV modificados, se recalculará en el próximo frame.");
                    break;
                }
            }

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    // --- Timer principal del pipeline ---
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(period, std::bind(&LanePipelineNode::process_next_frame, this));

    RCLCPP_INFO(get_logger(), "LanePipelineNode iniciado. Rate: %.1f Hz | paused=%s", rate_hz, paused_ ? "true" : "false");
}
