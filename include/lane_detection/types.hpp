#pragma once

#include <cmath>
#include <string>
#include <vector>

/**
 * types.hpp
 * ---------
 * Estructuras compartidas por todos los componentes del pipeline.
 */

// Coeficientes del polinomio  x = a·y² + b·y + c
struct PolyCoeffs {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    bool valid = false;
};

// Estado de detección de carril (izquierda, derecha, centro)
struct LaneState {
    PolyCoeffs left;
    PolyCoeffs right;
    PolyCoeffs center;
    bool width_warning = false;  // ancho fuera de [0.25, 0.45] m
};

// Errores para el controlador MPC
struct LaneErrors {
    double e2 = 0.0;   // [m]   error lateral
    double e3 = 0.0;   // [rad] error angular
    double k  = 0.0;   // [m⁻¹] curvatura
    bool valid = false;
};

// Configuración de la transformación BEV
struct BevConfig {
    // Modo de homografía: "points" (4 puntos manuales) o "model" (desde cámara)
    std::string mode = "points";

    // --- Modo "points" ---
    std::vector<double> src_pts = {
        -1.0, 274.0,
        652.0, 275.0,
        822.0, 344.0,
        -159.0, 344.0,
    };
    std::vector<double> dst_pts = {
        320.0 * 0.10,   0.0,            //P1
        320.0 * 0.90,   0.0,            //P2
        320.0 * 0.90, 240.0,            //P3
        320.0 * 0.10, 240.0,            //P4
    };

    // --- Modo "model" (orientación de cámara) --- TODO: implementar
    double camera_pitch   = -8.0 * M_PI / 180.0;   // rad (-8°)
    double camera_yaw     =  0.0;   // rad (0°)
    double camera_roll    =  0.0;   // rad (0°)
    double camera_height  =  0.23;     // m
    double camera_lateral_offset = 0.0325;  // m (32.5 mm desde centro)

    // Intrínsecas (640×360, RealSense D435)
    double camera_fx = 459.304;
    double camera_fy = 457.987;
    double camera_cx = 315.689;
    double camera_cy = 178.333;

    // Imagen de entrada
    int img_w = 640;
    int img_h = 360;

    // BEV de salida
    int bev_w = 320;
    int bev_h = 240;
    double bev_scale_mpp = 0.005;  // m/px

    // Offset longitudinal cámara → eje trasero
    double camera_longitudinal_offset_m = 0.23;  // m
};
