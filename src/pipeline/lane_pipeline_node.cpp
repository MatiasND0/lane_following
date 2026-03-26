/**
 * lane_pipeline_node.cpp
 * ----------------------
 * Implementación principal del nodo de detección de carril.
 *
 * Nota de arquitectura:
 * - Este archivo declara estructuras, clase y estado compartido.
 * - Las definiciones se segmentan por etapas en src/pipeline/stages/xx_*.cpp
 * - No se usan .hpp por requerimiento del proyecto.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <opencv2/opencv.hpp>

#include <atomic>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Estructuras de datos compartidas del pipeline
// ---------------------------------------------------------------------------

struct PolyCoeffs {
    double a = 0.0;  // coeficiente cuadrático  x = a*y^2 + b*y + c
    double b = 0.0;  // coeficiente lineal
    double c = 0.0;  // término independiente
    bool valid = false;
};

struct LaneState {
    PolyCoeffs left;
    PolyCoeffs right;
    PolyCoeffs center;
};

// ---------------------------------------------------------------------------
// Clase principal del nodo
// ---------------------------------------------------------------------------

class LanePipelineNode : public rclcpp::Node {
public:
    LanePipelineNode();

private:
    // 01. Inicialización
    void init_bev_homography();

    // 02. Fuente de frames
    void load_frame_list();
    void process_next_frame();
    void on_control_cmd(const std_msgs::msg::String::SharedPtr msg);

    // 03. Preprocesamiento HLS
    cv::Mat compute_hls_mask(const cv::Mat& bgr);

    // 04. Sliding window + fit polinomial
    LaneState run_sliding_window(const cv::Mat& bev_binary, cv::Mat& viz);
    PolyCoeffs fit_polynomial(const std::vector<cv::Point2f>& pts, int img_height);
    void draw_polynomial(cv::Mat& viz, const PolyCoeffs& c, int height, cv::Scalar color);

    // 05. Filtro temporal
    void apply_temporal_filter(const LaneState& raw);

    // 06. Errores para MPC
    void publish_lane_errors(const LaneState& state);

    // 07. Overlay y publicación debug
    void draw_lane_overlay(cv::Mat& bgr, const LaneState& state);
    void publish_image(
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& pub,
        const cv::Mat& img,
        const std::string& label);

private:
    // Parámetros
    std::string frames_dir_;
    double bev_scale_mpp_ = 0.005;
    double alpha_filter_ = 0.3;
    double lane_width_m_ = 0.35;
    double camera_offset_m_ = 0.23;
    int bev_w_ = 320;
    int bev_h_ = 240;

    // Homografías
    cv::Mat H_;
    cv::Mat H_inv_;
    std::atomic<bool> bev_dirty_{false};

    // Callback de parámetros dinámicos
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Estado persistente
    std::vector<std::string> frame_paths_;
    int frame_idx_ = 0;
    LaneState state_filtered_;

    // Publicadores
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_original_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_hls_mask_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_bev_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_sliding_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_overlay_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_errors_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_control_cmd_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Control de reproducción offline
    bool paused_ = false;
    bool step_once_ = false;
};

// ---------------------------------------------------------------------------
// Factory pública usada por src/main.cpp (sin headers)
// ---------------------------------------------------------------------------

std::shared_ptr<rclcpp::Node> make_lane_pipeline_node();

// ---------------------------------------------------------------------------
// Implementación por etapas (archivos incluidos por requerimiento)
// ---------------------------------------------------------------------------

#include "stages/01_params_and_init.cpp"
#include "stages/02_frame_source.cpp"
#include "stages/03_preprocess_hls.cpp"
#include "stages/04_bev_transform.cpp"
#include "stages/05_sliding_window.cpp"
#include "stages/06_temporal_filter.cpp"
#include "stages/07_errors_mpc.cpp"
#include "stages/08_debug_publish.cpp"

std::shared_ptr<rclcpp::Node> make_lane_pipeline_node() {
    return std::make_shared<LanePipelineNode>();
}
