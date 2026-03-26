#pragma once

#include "lane_detection/types.hpp"

/**
 * lane_tracker.hpp
 * -----------------
 * Filtrado temporal exponencial de coeficientes polinomiales.
 * Stateful: guarda LaneState prev_ entre llamadas.
 */

namespace lane_detection {

class LaneTracker {
public:
    /// Constructor con alpha del filtro EMA.
    explicit LaneTracker(double alpha = 0.3);

    /// Aplica filtro EMA y rechaza saltos espurios de curvatura.
    /// Si width_warning == true en raw, usa alpha/2 en lugar de alpha.
    void update(const LaneState& raw);

    /// Estado filtrado actual.
    const LaneState& state() const { return prev_; }

    /// Reset del estado.
    void reset() { prev_ = LaneState{}; }

    /// Cambiar alpha (para reconfiguración dinámica).
    void set_alpha(double alpha) { alpha_ = alpha; }

private:
    LaneState prev_;
    double alpha_;
};

}  // namespace lane_detection
