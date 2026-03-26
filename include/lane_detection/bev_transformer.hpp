#pragma once

#include <opencv2/core.hpp>

#include "lane_detection/types.hpp"

/**
 * bev_transformer.hpp
 * --------------------
 * Homografía BEV.
 * Modo "points": 4 pares src/dst (calibración manual).
 * Modo "model": TODO — modelo analítico desde orientación de cámara.
 */

namespace lane_detection {

class BevTransformer {
public:
    /// Constructor: precalcula H_ y H_inv_ según cfg.
    explicit BevTransformer(const BevConfig& cfg);

    /// Aplica warpPerspective sobre la máscara binaria.
    cv::Mat transform(const cv::Mat& binary) const;

    /// Proyecta el polinomio central de BEV a la imagen perspectiva original.
    /// Devuelve una imagen BGR con el overlay dibujado (mismo tamaño que img_w × img_h).
    cv::Mat project_to_image(const LaneState& state, const cv::Size& img_size) const;

    /// Recalcula H_ y H_inv_ (para reconfiguración dinámica).
    void reconfigure(const BevConfig& cfg);

    cv::Mat H()     const { return H_; }
    cv::Mat H_inv() const { return H_inv_; }

private:
    cv::Mat H_;
    cv::Mat H_inv_;
    int bev_w_ = 320;
    int bev_h_ = 240;

    void compute_from_points(const BevConfig& cfg);

    // TODO: void compute_from_model(const BevConfig& cfg);
};

}  // namespace lane_detection
