/**
 * lane_pipeline_node.cpp
 * ----------------------
 * Nodo ROS2 de detección de carril.
 * Solo contiene: parámetros, publishers, timer y orquestación.
 * No contiene lógica de visión ni matemática (excepto cv::imread).
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <opencv2/imgcodecs.hpp>    // cv::imread, cv::imencode
#include <opencv2/imgproc.hpp>      // cv::cvtColor (para convertir gray→BGR de debug)

#include <atomic>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "lane_detection/types.hpp"
#include "lane_detection/preprocessor.hpp"
#include "lane_detection/bev_transformer.hpp"
#include "lane_detection/sliding_window.hpp"
#include "lane_detection/lane_tracker.hpp"
#include "lane_detection/error_calculator.hpp"

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Clase principal del nodo
// ---------------------------------------------------------------------------

class LanePipelineNode : public rclcpp::Node {
public:
    LanePipelineNode();

private:
    // Fuente de frames
    void load_frame_list();
    void process_next_frame();
    void on_control_cmd(const std_msgs::msg::String::SharedPtr msg);

    // Publicación de imágenes debug
    void publish_image(
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& pub,
        const cv::Mat& img,
        const std::string& label);

    // Construye configs desde parámetros ROS2
    BevConfig                        build_bev_config() const;
    lane_detection::HlsParams        build_hls_params() const;
    lane_detection::SlidingWindowParams build_sw_params() const;

private:
    // Parámetros cacheados
    std::string frames_dir_;
    double bev_scale_mpp_;
    double lane_width_m_;
    int bev_w_ = 320;
    int bev_h_ = 240;

    // Flag para recalcular homografía
    std::atomic<bool> bev_dirty_{false};

    // Callback de parámetros dinámicos
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // Componentes del pipeline
    std::unique_ptr<lane_detection::BevTransformer> bev_transformer_;
    lane_detection::LaneTracker lane_tracker_;

    // Configs actuales
    lane_detection::HlsParams hls_params_;
    lane_detection::SlidingWindowParams sw_params_;

    // Estado de reproducción offline
    std::vector<std::string> frame_paths_;
    int frame_idx_ = 0;
    bool paused_ = false;
    bool step_once_ = false;

    // Publicadores
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_original_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_hls_mask_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_bev_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_sliding_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_overlay_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  pub_errors_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_control_cmd_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // BevConfig cacheado para error_calculator
    BevConfig bev_cfg_;
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LanePipelineNode::LanePipelineNode()
    : Node("lane_pipeline_node"),
      lane_tracker_(0.3)
{
    // --- Parámetros generales ---
    declare_parameter("frames_dir", std::string(""));
    declare_parameter("publish_rate_hz", 25.0);
    declare_parameter("bev_scale_mpp", 0.005);
    declare_parameter("alpha_filter", 0.3);
    declare_parameter("lane_width_m", 0.35);
    declare_parameter("camera_offset_m", 0.23);
    declare_parameter("start_paused", false);

    // --- Homografía BEV: modo "points" ---
    declare_parameter("bev_src_pts", std::vector<double>{
        640.0 * 0.40, 360.0 * 0.55,
        640.0 * 0.60, 360.0 * 0.55,
        640.0 * 0.95, 360.0 * 0.95,
        640.0 * 0.05, 360.0 * 0.95,
    });
    declare_parameter("bev_dst_pts", std::vector<double>{
        320.0 * 0.10,   0.0,
        320.0 * 0.90,   0.0,
        320.0 * 0.90, 240.0,
        320.0 * 0.10, 240.0,
    });

    // --- Parámetros de cámara (para modo "model" — TODO) ---
    declare_parameter("bev_mode", std::string("points"));
    declare_parameter("camera_pitch", -8.0 * M_PI / 180.0);
    declare_parameter("camera_yaw", 0.0);
    declare_parameter("camera_roll", 0.0);
    declare_parameter("camera_height", 0.23);
    declare_parameter("camera_lateral_offset", 0.0325);
    declare_parameter("camera_fx", 459.304);
    declare_parameter("camera_fy", 457.987);
    declare_parameter("camera_cx", 315.689);
    declare_parameter("camera_cy", 178.333);

    // --- Leer parámetros ---
    frames_dir_    = get_parameter("frames_dir").as_string();
    bev_scale_mpp_ = get_parameter("bev_scale_mpp").as_double();
    lane_width_m_  = get_parameter("lane_width_m").as_double();
    paused_        = get_parameter("start_paused").as_bool();

    const double alpha   = get_parameter("alpha_filter").as_double();
    const double rate_hz = get_parameter("publish_rate_hz").as_double();

    // Configurar tracker
    lane_tracker_ = lane_detection::LaneTracker(alpha);

    if (frames_dir_.empty()) {
        RCLCPP_ERROR(get_logger(), "Parámetro 'frames_dir' no seteado. Uso:");
        RCLCPP_ERROR(get_logger(),
            "  ros2 run lane_detection lane_pipeline_node --ros-args -p frames_dir:=/path/to/frames");
        rclcpp::shutdown();
        return;
    }

    // --- Fuente offline ---
    load_frame_list();
    if (frame_paths_.empty()) {
        RCLCPP_ERROR(get_logger(),
            "No se encontraron imágenes .jpg/.png en: %s", frames_dir_.c_str());
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(get_logger(), "Cargados %zu frames desde %s",
                frame_paths_.size(), frames_dir_.c_str());

    // --- Publishers de debug ---
    const auto qos = rclcpp::QoS(10);
    pub_original_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/original", qos);
    pub_hls_mask_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/hls_mask", qos);
    pub_bev_      = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/bev", qos);
    pub_sliding_  = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/sliding_window", qos);
    pub_overlay_  = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/overlay", qos);
    pub_errors_   = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_errors_est", qos);

    // --- Control de reproducción ---
    sub_control_cmd_ = create_subscription<std_msgs::msg::String>(
        "/lane_control/cmd", qos,
        std::bind(&LanePipelineNode::on_control_cmd, this, std::placeholders::_1));

    // --- Construir configs e inicializar componentes ---
    bev_cfg_    = build_bev_config();
    hls_params_ = build_hls_params();
    sw_params_  = build_sw_params();
    bev_transformer_ = std::make_unique<lane_detection::BevTransformer>(bev_cfg_);

    // --- Callback de reconfiguración dinámica ---
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
            -> rcl_interfaces::msg::SetParametersResult
        {
            for (const auto& p : params) {
                const auto& name = p.get_name();
                if (name == "bev_src_pts" || name == "bev_dst_pts" ||
                    name == "bev_mode" ||
                    name == "camera_pitch" || name == "camera_yaw" ||
                    name == "camera_roll" || name == "camera_height" ||
                    name == "camera_lateral_offset") {
                    bev_dirty_.store(true);
                    RCLCPP_INFO(get_logger(),
                        "Parámetro BEV '%s' modificado, se recalculará.", name.c_str());
                    break;
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    // --- Timer principal ---
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(period,
        std::bind(&LanePipelineNode::process_next_frame, this));

    RCLCPP_INFO(get_logger(),
        "LanePipelineNode iniciado. Rate: %.1f Hz | paused=%s",
        rate_hz, paused_ ? "true" : "false");
}

// ---------------------------------------------------------------------------
// Builders de config desde parámetros ROS2
// ---------------------------------------------------------------------------

BevConfig LanePipelineNode::build_bev_config() const {
    BevConfig cfg;
    cfg.mode    = get_parameter("bev_mode").as_string();
    cfg.src_pts = get_parameter("bev_src_pts").as_double_array();
    cfg.dst_pts = get_parameter("bev_dst_pts").as_double_array();
    cfg.camera_pitch  = get_parameter("camera_pitch").as_double();
    cfg.camera_yaw    = get_parameter("camera_yaw").as_double();
    cfg.camera_roll   = get_parameter("camera_roll").as_double();
    cfg.camera_height = get_parameter("camera_height").as_double();
    cfg.camera_lateral_offset = get_parameter("camera_lateral_offset").as_double();
    cfg.camera_fx = get_parameter("camera_fx").as_double();
    cfg.camera_fy = get_parameter("camera_fy").as_double();
    cfg.camera_cx = get_parameter("camera_cx").as_double();
    cfg.camera_cy = get_parameter("camera_cy").as_double();
    cfg.bev_w = bev_w_;
    cfg.bev_h = bev_h_;
    cfg.bev_scale_mpp = bev_scale_mpp_;
    cfg.camera_longitudinal_offset_m = get_parameter("camera_offset_m").as_double();
    return cfg;
}

lane_detection::HlsParams LanePipelineNode::build_hls_params() const {
    // Valores por defecto; en el futuro se pueden exponer como parámetros ROS2
    return lane_detection::HlsParams{};
}

lane_detection::SlidingWindowParams LanePipelineNode::build_sw_params() const {
    lane_detection::SlidingWindowParams p;
    p.lane_width_m  = lane_width_m_;
    p.bev_scale_mpp = bev_scale_mpp_;
    return p;
}

// ---------------------------------------------------------------------------
// Fuente de frames
// ---------------------------------------------------------------------------

void LanePipelineNode::load_frame_list() {
    for (const auto& entry : fs::directory_iterator(frames_dir_)) {
        const auto ext = entry.path().extension();
        if (ext == ".jpg" || ext == ".png") {
            frame_paths_.push_back(entry.path().string());
        }
    }
    std::sort(frame_paths_.begin(), frame_paths_.end());
}

// ---------------------------------------------------------------------------
// Ciclo principal
// ---------------------------------------------------------------------------

void LanePipelineNode::process_next_frame() {
    // Recalcular homografía si los parámetros cambiaron
    if (bev_dirty_.exchange(false)) {
        bev_cfg_ = build_bev_config();
        bev_transformer_->reconfigure(bev_cfg_);
        RCLCPP_INFO(get_logger(), "Homografía BEV recalculada.");
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

    // 2) HLS mask
    cv::Mat hls_mask = lane_detection::compute_mask(frame, hls_params_);
    cv::Mat hls_mask_viz;
    cv::cvtColor(hls_mask, hls_mask_viz, cv::COLOR_GRAY2BGR);
    publish_image(pub_hls_mask_, hls_mask_viz, "hls_mask");

    // 3) BEV
    cv::Mat bev_binary = bev_transformer_->transform(hls_mask);
    cv::Mat bev_viz;
    cv::cvtColor(bev_binary, bev_viz, cv::COLOR_GRAY2BGR);
    publish_image(pub_bev_, bev_viz, "bev");

    // 4) Sliding window + fit
    cv::Mat sliding_viz = bev_viz.clone();
    const LaneState raw_state = lane_detection::detect_lanes(
        bev_binary, sliding_viz, bev_w_, bev_h_, sw_params_);
    publish_image(pub_sliding_, sliding_viz, "sliding_window");

    // 5) Filtro temporal
    lane_tracker_.update(raw_state);

    // 6) Overlay
    cv::Mat overlay_layer = bev_transformer_->project_to_image(
        lane_tracker_.state(), frame.size());
    cv::Mat overlay = frame.clone();
    cv::add(overlay, overlay_layer, overlay);
    publish_image(pub_overlay_, overlay, "overlay");

    // 7) Salida al MPC
    const auto errors = lane_detection::compute_lane_errors(
        lane_tracker_.state(), bev_cfg_);

    if (errors.valid) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            static_cast<float>(errors.e2),
            static_cast<float>(errors.e3),
            static_cast<float>(errors.k)
        };
        pub_errors_->publish(msg);
        RCLCPP_DEBUG(get_logger(), "e2=%.4f m  e3=%.4f rad  k=%.4f m^-1",
                     errors.e2, errors.e3, errors.k);
    }
}

// ---------------------------------------------------------------------------
// Control de reproducción
// ---------------------------------------------------------------------------

void LanePipelineNode::on_control_cmd(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& cmd = msg->data;

    if (cmd == "pause") {
        paused_ = true;
        RCLCPP_INFO(get_logger(), "Control: pause");
    } else if (cmd == "resume") {
        paused_ = false;
        RCLCPP_INFO(get_logger(), "Control: resume");
    } else if (cmd == "toggle") {
        paused_ = !paused_;
        RCLCPP_INFO(get_logger(), "Control: toggle -> paused=%s",
                     paused_ ? "true" : "false");
    } else if (cmd == "step") {
        paused_ = true;
        step_once_ = true;
        RCLCPP_DEBUG(get_logger(), "Control: step");
    } else if (cmd == "back") {
        paused_ = true;
        step_once_ = true;
        frame_idx_ = std::max(0, frame_idx_ - 2);
        RCLCPP_INFO(get_logger(), "Control: back (frame_idx=%d)", frame_idx_);
    } else if (cmd == "reset") {
        frame_idx_ = 0;
        lane_tracker_.reset();
        RCLCPP_INFO(get_logger(), "Control: reset (frame_idx=0)");
    } else {
        RCLCPP_WARN(get_logger(),
            "Control desconocido: '%s'", cmd.c_str());
    }
}

// ---------------------------------------------------------------------------
// Publicación de imagen comprimida
// ---------------------------------------------------------------------------

void LanePipelineNode::publish_image(
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& pub,
    const cv::Mat& img,
    const std::string& label)
{
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = now();
    msg.header.frame_id = label;
    msg.format = "jpeg";

    std::vector<uchar> buf;
    cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, 85});
    msg.data.assign(buf.begin(), buf.end());

    pub->publish(msg);
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

std::shared_ptr<rclcpp::Node> make_lane_pipeline_node() {
    return std::make_shared<LanePipelineNode>();
}
