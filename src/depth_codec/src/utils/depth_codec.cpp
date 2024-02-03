#include "depth_codec/depth_codec.hpp"  // Asegúrate de que el include sea correcto
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

std::shared_ptr<sensor_msgs::msg::Image> to_code_frame(const sensor_msgs::msg::Image::SharedPtr& frame) {
    // Convertir el mensaje ROS a una imagen OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::MONO16);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return nullptr;
    }

    // Procesar la imagen OpenCV (por ejemplo, invertir colores)
    cv::Mat &mat = cv_ptr->image;
    mat = 65535 - mat;  // Invertir colores para una imagen de 16 bits

    // Convertir la imagen OpenCV procesada de vuelta a un mensaje ROS
    auto processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", mat).toImageMsg();

    return processed_image_msg;
}


std::shared_ptr<sensor_msgs::msg::Image> to_decode_frame(const sensor_msgs::msg::Image::SharedPtr& frame) {
    // Convertir el mensaje ROS a una imagen OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::MONO16);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return nullptr;
    }

    // Procesar la imagen OpenCV (por ejemplo, invertir colores)
    cv::Mat &mat = cv_ptr->image;
    mat = 65535 - mat;  // Invertir colores para una imagen de 16 bits

    // Convertir la imagen OpenCV procesada de vuelta a un mensaje ROS
    auto processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", mat).toImageMsg();

    return processed_image_msg;
}