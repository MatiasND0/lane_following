/**
 * Etapa 08: utilidades de publicación de imágenes de debug.
 */

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
