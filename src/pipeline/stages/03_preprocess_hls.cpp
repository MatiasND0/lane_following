/**
 * Etapa 03: preprocesamiento HLS y máscara binaria de líneas.
 */

cv::Mat LanePipelineNode::compute_hls_mask(const cv::Mat& bgr) {
    cv::Mat hls;
    cv::GaussianBlur(bgr, hls, cv::Size(5, 5), 0);
    cv::cvtColor(hls, hls, cv::COLOR_BGR2HLS);

    // Líneas blancas: L alto, S bajo
    cv::Mat white_mask;
    cv::inRange(hls, cv::Scalar(0, 180, 0), cv::Scalar(180, 255, 60), white_mask);

    // Líneas amarillas: H amarillo, S alto
    cv::Mat yellow_mask;
    cv::inRange(hls, cv::Scalar(15, 80, 100), cv::Scalar(35, 220, 255), yellow_mask);

    // Fusión
    cv::Mat combined;
    cv::bitwise_or(white_mask, yellow_mask, combined);

    // Cierre morfológico para continuidad de línea
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, kernel);

    return combined;
}
