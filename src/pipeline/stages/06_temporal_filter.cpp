/**
 * Etapa 06: filtrado temporal exponencial de coeficientes polinomiales.
 */

void LanePipelineNode::apply_temporal_filter(const LaneState& raw) {
    auto filter_coeff = [](PolyCoeffs& filtered, const PolyCoeffs& raw_c, double alpha) {
        if (!raw_c.valid) {
            return;
        }

        if (!filtered.valid) {
            filtered = raw_c;
            return;
        }

        // Rechazo de salto espurio de curvatura.
        if (std::abs(raw_c.a) > 0.0 &&
            std::abs(raw_c.a - filtered.a) / (std::abs(filtered.a) + 1e-9) > 1.5) {
            return;
        }

        filtered.a = alpha * raw_c.a + (1.0 - alpha) * filtered.a;
        filtered.b = alpha * raw_c.b + (1.0 - alpha) * filtered.b;
        filtered.c = alpha * raw_c.c + (1.0 - alpha) * filtered.c;
    };

    filter_coeff(state_filtered_.left, raw.left, alpha_filter_);
    filter_coeff(state_filtered_.right, raw.right, alpha_filter_);
    filter_coeff(state_filtered_.center, raw.center, alpha_filter_);
}
