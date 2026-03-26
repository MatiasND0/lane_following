/**
 * lane_pipeline_node.cpp
 * ----------------------
 * Nodo ROS2 (C++) que implementa el pipeline completo de detección de carril.
 *
 * Etapas del pipeline:
 *   1. Lectura de imagen desde carpeta (simula source offline)
 *   2. Preprocesamiento HLS + umbralización binaria
 *   3. Transformación Bird's Eye View (IPM)
 *   4. Sliding window + ajuste polinomial grado 2
 *   5. Filtrado temporal exponencial sobre coeficientes
 *   6. Cálculo de e2, e3, k → publicado en /lane_errors_est
 *
 * Topics publicados (debug):
 *   /lane_debug/original           sensor_msgs/CompressedImage
 *   /lane_debug/hls_mask           sensor_msgs/CompressedImage
 *   /lane_debug/bev                sensor_msgs/CompressedImage
 *   /lane_debug/sliding_window     sensor_msgs/CompressedImage
 *   /lane_debug/overlay            sensor_msgs/CompressedImage
 *
 * Topic publicado (salida):
 *   /lane_errors_est               std_msgs/Float32MultiArray  [e2, e3, k]
 *
 * Parámetros ROS2 (seteables desde launch o CLI):
 *   frames_dir       : ruta a la carpeta con los .jpg extraídos
 *   publish_rate_hz  : frecuencia de procesamiento (default: 25.0)
 *   bev_scale_mpp    : metros por píxel en BEV (default: 0.005)
 *   alpha_filter     : coeficiente filtro exponencial (default: 0.3)
 *   lane_width_m     : ancho nominal del carril en metros (default: 0.35)
 *   camera_offset_m  : offset cámara respecto al eje trasero (default: 0.23)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Estructuras de datos
// ---------------------------------------------------------------------------

struct PolyCoeffs {
    double a = 0.0;  // coeficiente cuadrático  x = a*y^2 + b*y + c
    double b = 0.0;  // coeficiente lineal
    double c = 0.0;  // término independiente (posición lateral en y=0)
    bool valid = false;
};

struct LaneState {
    PolyCoeffs left;
    PolyCoeffs right;
    PolyCoeffs center;
};

// ---------------------------------------------------------------------------
// Nodo principal
// ---------------------------------------------------------------------------

class LanePipelineNode : public rclcpp::Node {
public:
    LanePipelineNode() : Node("lane_pipeline_node") {
        // --- Parámetros ---
        declare_parameter("frames_dir", std::string(""));
        declare_parameter("publish_rate_hz", 25.0);
        declare_parameter("bev_scale_mpp", 0.005);   // 5 mm/px
        declare_parameter("alpha_filter", 0.3);
        declare_parameter("lane_width_m", 0.35);
        declare_parameter("camera_offset_m", 0.23);

        frames_dir_     = get_parameter("frames_dir").as_string();
        bev_scale_mpp_  = get_parameter("bev_scale_mpp").as_double();
        alpha_filter_   = get_parameter("alpha_filter").as_double();
        lane_width_m_   = get_parameter("lane_width_m").as_double();
        camera_offset_m_= get_parameter("camera_offset_m").as_double();
        double rate_hz  = get_parameter("publish_rate_hz").as_double();

        if (frames_dir_.empty()) {
            RCLCPP_ERROR(get_logger(), "Parámetro 'frames_dir' no seteado. Uso:");
            RCLCPP_ERROR(get_logger(), "  ros2 run lane_detection lane_pipeline_node --ros-args -p frames_dir:=/path/to/frames");
            rclcpp::shutdown();
            return;
        }

        // --- Cargar lista de frames ---
        load_frame_list();
        if (frame_paths_.empty()) {
            RCLCPP_ERROR(get_logger(), "No se encontraron imágenes .jpg en: %s", frames_dir_.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Cargados %zu frames desde %s", frame_paths_.size(), frames_dir_.c_str());

        // --- Publishers debug ---
        auto qos = rclcpp::QoS(10);
        pub_original_  = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/original",        qos);
        pub_hls_mask_  = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/hls_mask",        qos);
        pub_bev_       = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/bev",             qos);
        pub_sliding_   = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/sliding_window",  qos);
        pub_overlay_   = create_publisher<sensor_msgs::msg::CompressedImage>("/lane_debug/overlay",         qos);

        // --- Publisher salida ---
        pub_errors_    = create_publisher<std_msgs::msg::Float32MultiArray>("/lane_errors_est", qos);

        // --- Timer de procesamiento ---
        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(period, std::bind(&LanePipelineNode::process_next_frame, this));

        // --- Inicializar homografía BEV ---
        init_bev_homography();

        RCLCPP_INFO(get_logger(), "LanePipelineNode iniciado. Rate: %.1f Hz", rate_hz);
    }

private:
    // -----------------------------------------------------------------------
    // Carga de frames
    // -----------------------------------------------------------------------

    void load_frame_list() {
        for (const auto& entry : fs::directory_iterator(frames_dir_)) {
            if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png") {
                frame_paths_.push_back(entry.path().string());
            }
        }
        std::sort(frame_paths_.begin(), frame_paths_.end());
    }

    // -----------------------------------------------------------------------
    // Inicialización de la homografía Bird's Eye View
    // -----------------------------------------------------------------------

    void init_bev_homography() {
        // Dimensiones de la imagen de entrada esperada (D435 a 640x480)
        // Puntos en imagen original (trapecio sobre el asfalto)
        // Estos valores son para calibración en BFMC - deben ajustarse con
        // la herramienta de calibración interactiva una vez disponible.
        //
        // El trapecio cubre la región de interés del suelo:
        //   - Fila superior: ~55% de la altura (elimina cielo)
        //   - Fila inferior: ~95% de la altura (cerca del vehículo)
        //   - Lados convergentes hacia el centro por perspectiva

        const int src_w = 640;
        const int src_h = 480;

        // Puntos fuente en imagen perspectiva (x, y)
        std::vector<cv::Point2f> src_pts = {
            {static_cast<float>(src_w * 0.40f), static_cast<float>(src_h * 0.55f)},  // top-left
            {static_cast<float>(src_w * 0.60f), static_cast<float>(src_h * 0.55f)},  // top-right
            {static_cast<float>(src_w * 0.95f), static_cast<float>(src_h * 0.95f)},  // bottom-right
            {static_cast<float>(src_w * 0.05f), static_cast<float>(src_h * 0.95f)},  // bottom-left
        };

        // Dimensiones de la imagen BEV de salida: 320x240 px @ 5mm/px
        // Cubre 1.6m lateral x 1.2m longitudinal
        bev_w_ = 320;
        bev_h_ = 240;

        // Puntos destino (rectángulo en BEV)
        std::vector<cv::Point2f> dst_pts = {
            {static_cast<float>(bev_w_ * 0.10f), 0.0f},
            {static_cast<float>(bev_w_ * 0.90f), 0.0f},
            {static_cast<float>(bev_w_ * 0.90f), static_cast<float>(bev_h_)},
            {static_cast<float>(bev_w_ * 0.10f), static_cast<float>(bev_h_)},
        };

        H_   = cv::getPerspectiveTransform(src_pts, dst_pts);
        H_inv_ = cv::getPerspectiveTransform(dst_pts, src_pts);
    }

    // -----------------------------------------------------------------------
    // Loop principal de procesamiento
    // -----------------------------------------------------------------------

    void process_next_frame() {
        if (frame_idx_ >= static_cast<int>(frame_paths_.size())) {
            RCLCPP_INFO_ONCE(get_logger(), "Todos los frames procesados.");
            return;
        }

        cv::Mat frame = cv::imread(frame_paths_[frame_idx_++]);
        if (frame.empty()) {
            RCLCPP_WARN(get_logger(), "Frame vacío en idx %d, saltando.", frame_idx_ - 1);
            return;
        }

        // ---- Etapa 1: Imagen original ----
        publish_image(pub_original_, frame, "original");

        // ---- Etapa 2: Máscara HLS ----
        cv::Mat hls_mask = compute_hls_mask(frame);
        cv::Mat hls_mask_viz;
        cv::cvtColor(hls_mask, hls_mask_viz, cv::COLOR_GRAY2BGR);
        publish_image(pub_hls_mask_, hls_mask_viz, "hls_mask");

        // ---- Etapa 3: Bird's Eye View ----
        cv::Mat bev_binary;
        cv::warpPerspective(hls_mask, bev_binary, H_, cv::Size(bev_w_, bev_h_));

        cv::Mat bev_viz;
        cv::cvtColor(bev_binary, bev_viz, cv::COLOR_GRAY2BGR);
        publish_image(pub_bev_, bev_viz, "bev");

        // ---- Etapa 4: Sliding window ----
        cv::Mat sliding_viz = bev_viz.clone();
        LaneState raw_state = run_sliding_window(bev_binary, sliding_viz);
        publish_image(pub_sliding_, sliding_viz, "sliding_window");

        // ---- Etapa 5: Filtrado temporal ----
        apply_temporal_filter(raw_state);

        // ---- Etapa 6: Overlay + cálculo de errores ----
        cv::Mat overlay = frame.clone();
        draw_lane_overlay(overlay, state_filtered_);
        publish_image(pub_overlay_, overlay, "overlay");

        // ---- Publicar errores para el MPC ----
        publish_lane_errors(state_filtered_);
    }

    // -----------------------------------------------------------------------
    // Etapa 2: Preprocesamiento HLS
    // -----------------------------------------------------------------------

    cv::Mat compute_hls_mask(const cv::Mat& bgr) {
        cv::Mat hls;
        cv::GaussianBlur(bgr, hls, cv::Size(5, 5), 0);
        cv::cvtColor(hls, hls, cv::COLOR_BGR2HLS);

        // Máscara líneas blancas: L alto, S bajo
        cv::Mat white_mask;
        cv::inRange(hls, cv::Scalar(0, 180, 0), cv::Scalar(180, 255, 60), white_mask);

        // Máscara líneas amarillas: H en rango amarillo, S alto
        cv::Mat yellow_mask;
        cv::inRange(hls, cv::Scalar(15, 80, 100), cv::Scalar(35, 220, 255), yellow_mask);

        // Fusión
        cv::Mat combined;
        cv::bitwise_or(white_mask, yellow_mask, combined);

        // Operaciones morfológicas para cerrar huecos en líneas discontinuas
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, kernel);

        return combined;
    }

    // -----------------------------------------------------------------------
    // Etapa 4: Sliding Window
    // -----------------------------------------------------------------------

    LaneState run_sliding_window(const cv::Mat& bev_binary, cv::Mat& viz) {
        const int n_windows = 9;
        const int win_half_w = 30;   // ±30 px = ±15 cm en el suelo
        const int min_pixels = 40;

        // Histograma en la mitad inferior de la imagen
        cv::Mat bottom_half = bev_binary(cv::Rect(0, bev_h_ / 2, bev_w_, bev_h_ / 2));
        cv::Mat hist;
        cv::reduce(bottom_half, hist, 0, cv::REDUCE_SUM, CV_32F);

        // Encontrar pico izquierdo y derecho
        int mid = bev_w_ / 2;
        cv::Point left_peak_loc, right_peak_loc;
        double dummy;
        cv::minMaxLoc(hist(cv::Rect(0, 0, mid, 1)),         nullptr, &dummy, nullptr, &left_peak_loc);
        cv::minMaxLoc(hist(cv::Rect(mid, 0, mid, 1)),       nullptr, &dummy, nullptr, &right_peak_loc);
        int cur_left_x  = left_peak_loc.x;
        int cur_right_x = right_peak_loc.x + mid;

        int win_h = bev_h_ / n_windows;

        std::vector<cv::Point2f> left_pts, right_pts;

        for (int win = 0; win < n_windows; ++win) {
            int y_low  = bev_h_ - (win + 1) * win_h;

            // Ventana izquierda
            auto extract_window = [&](int cx, std::vector<cv::Point2f>& pts, cv::Scalar color) {
                int x_low  = std::max(0,       cx - win_half_w);
                int x_high = std::min(bev_w_,  cx + win_half_w);
                cv::Rect roi(x_low, y_low, x_high - x_low, win_h);
                cv::rectangle(viz, roi, color, 1);

                cv::Mat window = bev_binary(roi);
                std::vector<cv::Point> nz;
                cv::findNonZero(window, nz);

                if (static_cast<int>(nz.size()) >= min_pixels) {
                    double sum_x = 0;
                    for (auto& p : nz) sum_x += p.x + x_low;
                    cx = static_cast<int>(sum_x / nz.size());
                    for (auto& p : nz) {
                        pts.emplace_back(static_cast<float>(p.x + x_low),
                                         static_cast<float>(p.y + y_low));
                    }
                }
                return cx;
            };

            cur_left_x  = extract_window(cur_left_x,  left_pts,  cv::Scalar(255, 100, 0));
            cur_right_x = extract_window(cur_right_x, right_pts, cv::Scalar(0, 100, 255));
        }

        LaneState state;
        state.left  = fit_polynomial(left_pts,  bev_h_);
        state.right = fit_polynomial(right_pts, bev_h_);

        // Centro como promedio de coeficientes
        if (state.left.valid && state.right.valid) {
            state.center.a     = (state.left.a + state.right.a) / 2.0;
            state.center.b     = (state.left.b + state.right.b) / 2.0;
            state.center.c     = (state.left.c + state.right.c) / 2.0;
            state.center.valid = true;
        } else if (state.left.valid) {
            // Inferir línea derecha por offset nominal
            double lane_px = lane_width_m_ / bev_scale_mpp_;
            state.center.a     = state.left.a;
            state.center.b     = state.left.b;
            state.center.c     = state.left.c + lane_px / 2.0;
            state.center.valid = true;
        } else if (state.right.valid) {
            double lane_px = lane_width_m_ / bev_scale_mpp_;
            state.center.a     = state.right.a;
            state.center.b     = state.right.b;
            state.center.c     = state.right.c - lane_px / 2.0;
            state.center.valid = true;
        }

        // Dibujar polinomios ajustados sobre la visualización
        draw_polynomial(viz, state.left,   bev_h_, cv::Scalar(255, 200, 0));
        draw_polynomial(viz, state.right,  bev_h_, cv::Scalar(0, 200, 255));
        draw_polynomial(viz, state.center, bev_h_, cv::Scalar(0, 255, 0));

        return state;
    }

    // -----------------------------------------------------------------------
    // Ajuste polinomial grado 2: x = a*y^2 + b*y + c
    // -----------------------------------------------------------------------

    PolyCoeffs fit_polynomial(const std::vector<cv::Point2f>& pts, int img_height) {
        PolyCoeffs coeffs;
        if (static_cast<int>(pts.size()) < 20) {
            return coeffs;  // válido = false
        }

        // Construir sistema de ecuaciones para ajuste por mínimos cuadrados
        // El polinomio es x = a*y^2 + b*y + c
        // Variables: [y^2, y, 1] → coeficientes [a, b, c]
        int n = static_cast<int>(pts.size());
        cv::Mat A(n, 3, CV_64F);
        cv::Mat B(n, 1, CV_64F);

        // Normalizar y al rango [0,1] para estabilidad numérica
        double y_scale = static_cast<double>(img_height);
        for (int i = 0; i < n; ++i) {
            double y = pts[i].y / y_scale;
            A.at<double>(i, 0) = y * y;
            A.at<double>(i, 1) = y;
            A.at<double>(i, 2) = 1.0;
            B.at<double>(i, 0) = pts[i].x;
        }

        cv::Mat result;
        bool ok = cv::solve(A, B, result, cv::DECOMP_SVD);
        if (!ok) return coeffs;

        // Desescalar coeficientes: a_real = a_norm / y_scale^2, b_real = b_norm / y_scale
        // (porque x(y) = a*(y/s)^2 + b*(y/s) + c = (a/s^2)*y^2 + (b/s)*y + c)
        coeffs.a     = result.at<double>(0, 0) / (y_scale * y_scale);
        coeffs.b     = result.at<double>(1, 0) / y_scale;
        coeffs.c     = result.at<double>(2, 0);
        coeffs.valid = true;

        return coeffs;
    }

    // -----------------------------------------------------------------------
    // Etapa 5: Filtrado temporal exponencial
    // -----------------------------------------------------------------------

    void apply_temporal_filter(const LaneState& raw) {
        auto filter_coeff = [&](PolyCoeffs& filtered, const PolyCoeffs& raw_c, double alpha) {
            if (!raw_c.valid) return;
            if (!filtered.valid) {
                filtered = raw_c;
                return;
            }
            // Validación: curvatura no debe saltar más del 50%
            if (std::abs(raw_c.a) > 0 &&
                std::abs(raw_c.a - filtered.a) / (std::abs(filtered.a) + 1e-9) > 1.5) {
                return;  // Rechazar detección espuria, mantener el estado anterior
            }
            filtered.a = alpha * raw_c.a + (1.0 - alpha) * filtered.a;
            filtered.b = alpha * raw_c.b + (1.0 - alpha) * filtered.b;
            filtered.c = alpha * raw_c.c + (1.0 - alpha) * filtered.c;
        };

        filter_coeff(state_filtered_.left,   raw.left,   alpha_filter_);
        filter_coeff(state_filtered_.right,  raw.right,  alpha_filter_);
        filter_coeff(state_filtered_.center, raw.center, alpha_filter_);
    }

    // -----------------------------------------------------------------------
    // Etapa 6: Cálculo de e2, e3, k
    // -----------------------------------------------------------------------

    void publish_lane_errors(const LaneState& state) {
        if (!state.center.valid) return;

        // Sistema de referencia: origen en eje trasero, y adelante, x izquierda
        // La cámara está a camera_offset_m_ metros adelante del eje trasero.
        // En coordenadas BEV, y=0 (base de la imagen) corresponde a la posición
        // de la cámara. Para referenciar al eje trasero, el punto de evaluación
        // del polinomio es y = -camera_offset_m_ / bev_scale_mpp_ (detrás de la cámara).

        double y_eval_px = -camera_offset_m_ / bev_scale_mpp_;

        // e2: error lateral en metros
        // x_centro(y_eval) en píxeles, convertido a metros
        // El origen lateral del carril es el centro de la imagen BEV
        double x_center_px = state.center.a * y_eval_px * y_eval_px
                           + state.center.b * y_eval_px
                           + state.center.c;
        double x_lane_center_m = (x_center_px - bev_w_ / 2.0) * bev_scale_mpp_;
        double e2 = -x_lane_center_m;  // signo: positivo si el vehículo está a la derecha del centro

        // e3: error angular en radianes
        // Tangente al polinomio: dx/dy = 2a*y + b
        double dxdy = 2.0 * state.center.a * y_eval_px + state.center.b;
        // dx/dy está en px/px (adimensional), convertir usando bev_scale_mpp_ cancela
        double e3 = std::atan(dxdy);  // en radianes

        // k: curvatura en m^-1
        // k = 2a / (1 + (dx/dy)^2)^(3/2)  [aproximado: k ≈ 2a cuando dxdy << 1]
        double k_px_inv = 2.0 * state.center.a / std::pow(1.0 + dxdy * dxdy, 1.5);
        double k = k_px_inv / bev_scale_mpp_;  // convertir de px^-1 a m^-1

        // Publicar [e2, e3, k]
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {static_cast<float>(e2),
                    static_cast<float>(e3),
                    static_cast<float>(k)};
        pub_errors_->publish(msg);

        RCLCPP_DEBUG(get_logger(), "e2=%.4f m  e3=%.4f rad  k=%.4f m^-1", e2, e3, k);
    }

    // -----------------------------------------------------------------------
    // Overlay en imagen perspectiva original
    // -----------------------------------------------------------------------

    void draw_lane_overlay(cv::Mat& bgr, const LaneState& state) {
        if (!state.center.valid) return;

        // Proyectar polinomio central de BEV a imagen original
        std::vector<cv::Point2f> bev_pts, orig_pts;
        for (int y = 0; y < bev_h_; y += 5) {
            double x = state.center.a * y * y + state.center.b * y + state.center.c;
            if (x >= 0 && x < bev_w_) {
                bev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
            }
        }
        if (bev_pts.empty()) return;

        cv::perspectiveTransform(bev_pts, orig_pts, H_inv_);

        // Dibujar polilínea sobre la imagen original
        std::vector<cv::Point> poly_pts;
        for (auto& p : orig_pts) {
            poly_pts.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
        }
        cv::polylines(bgr, poly_pts, false, cv::Scalar(0, 255, 0), 3);
    }

    // -----------------------------------------------------------------------
    // Utilidades de visualización
    // -----------------------------------------------------------------------

    void draw_polynomial(cv::Mat& viz, const PolyCoeffs& c, int height, cv::Scalar color) {
        if (!c.valid) return;
        std::vector<cv::Point> pts;
        for (int y = 0; y < height; y += 2) {
            int x = static_cast<int>(c.a * y * y + c.b * y + c.c);
            if (x >= 0 && x < viz.cols) {
                pts.emplace_back(x, y);
            }
        }
        if (pts.size() > 1) {
            cv::polylines(viz, pts, false, color, 2);
        }
    }

    void publish_image(
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

    // -----------------------------------------------------------------------
    // Miembros privados
    // -----------------------------------------------------------------------

    // Parámetros
    std::string frames_dir_;
    double bev_scale_mpp_;
    double alpha_filter_;
    double lane_width_m_;
    double camera_offset_m_;
    int bev_w_, bev_h_;

    // Homografía
    cv::Mat H_, H_inv_;

    // Estado del pipeline
    std::vector<std::string> frame_paths_;
    int frame_idx_ = 0;
    LaneState state_filtered_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_original_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_hls_mask_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_bev_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_sliding_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_overlay_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_errors_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LanePipelineNode>());
    rclcpp::shutdown();
    return 0;
}