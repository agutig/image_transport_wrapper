#include "advanced_depth_codec/advanced_depth_codec.hpp"  // Asegúrate de que el include sea correcto
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

cv::Mat DCT_coding(const cv::Mat& inputImage, float compression_k) {
    // Ensure the image is in mono16 format
    if (inputImage.type() != CV_16UC1) {
        throw std::runtime_error("Unsupported image type. Expected mono16.");
    }

    // Convert the image to floating-point for DCT processing
    cv::Mat floatImage;
    inputImage.convertTo(floatImage, CV_32F);

    // Apply DCT
    cv::Mat dctImage;
    cv::dct(floatImage, dctImage);

    // Calcular las dimensiones para mantener los coeficientes
    int keepWidth = dctImage.cols * 1 / compression_k;
    int keepHeight = dctImage.rows * 1 / compression_k;

    cv::Mat mask = cv::Mat::zeros(dctImage.size(), CV_32F);
    cv::Mat roi(mask, cv::Rect(0, 0, keepWidth, keepHeight));
    roi.setTo(cv::Scalar(1));
    dctImage = dctImage.mul(mask);

    // Apply IDCT to reconstruct the image
    cv::Mat processedImage;
    cv::idct(dctImage, processedImage);

    // Convert the processed image back to mono16
    cv::Mat finalImage;
    processedImage.convertTo(finalImage, CV_16U);

    return finalImage;
}

std::shared_ptr<sensor_msgs::msg::Image> to_code_frame(const sensor_msgs::msg::Image::SharedPtr& frame , float compression_k) {
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
    //mat = 65535 - mat;  // Invertir colores para una imagen de 16 bits
    mat = DCT_coding(mat, compression_k);

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
    //mat = 65535 - mat;  // Invertir colores para una imagen de 16 bits
    
    // Convertir la imagen OpenCV procesada de vuelta a un mensaje ROS
    auto processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", mat).toImageMsg();

    return processed_image_msg;
}




