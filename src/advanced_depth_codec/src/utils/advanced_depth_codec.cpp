#include "advanced_depth_codec/advanced_depth_codec.hpp"  // Asegúrate de que el include sea correcto
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/rleimg.hpp"

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

coded_interfaces::msg::Rleimg DCT_to_RLE(const cv::Mat& dctImage) {
    // Esta función asume que dctImage es una matriz cuadrada
    int n = dctImage.rows; // Asumiendo que dctImage es cuadrada

    std::vector<float> rle_values;
    std::vector<int> rle_counts;
    float previousValue = 0;
    int runLength = 0;

    int i = 0, j = 0;
    int direction = 1; // 1 para arriba, -1 para abajo

    // Recorrido en zigzag
    for (int k = 0; k < n * n; ++k) {
        float currentValue = dctImage.at<float>(i, j);

        // Si es el primer valor o es diferente al valor anterior, se inicia un nuevo conteo
        if (k == 0 || currentValue != previousValue) {
            if (k != 0) {
                rle_values.push_back(previousValue);  // Añadir el valor al vector de valores RLE
                rle_counts.push_back(runLength);      // Añadir el conteo al vector de conteos RLE
            }
            previousValue = currentValue;
            runLength = 1;
        } else {
            // Si es el mismo valor, incrementar el contador
            runLength++;
        }

        // Moverse en zigzag
        if (direction == 1) {
            if (j == n - 1) { i++, direction = -1; } // Cambio de dirección en la última columna
            else if (i == 0) { j++, direction = -1; } // Cambio de dirección en la primera fila
            else { i--, j++; }
        } else {
            if (i == n - 1) { j++, direction = 1; } // Cambio de dirección en la última fila
            else if (j == 0) { i++, direction = 1; } // Cambio de dirección en la primera columna
            else { i++, j--; }
        }

        // Si es el último valor, asegurarse de añadirlo a los vectores RLE
        if (k == n * n - 1) {
            rle_values.push_back(currentValue);  // Añadir el valor al vector de valores RLE
            rle_counts.push_back(runLength);     // Añadir el conteo al vector de conteos RLE
        }
    }

    // Construir el mensaje RLEImage con los vectores y dimensiones originales
    coded_interfaces::msg::Rleimg rle_image_msg;
    rle_image_msg.original_width = n;
    rle_image_msg.original_height = n; // O utiliza la altura real si la imagen no es cuadrada
    rle_image_msg.rle_values = rle_values;
    rle_image_msg.rle_counts = rle_counts;

    return rle_image_msg;
}

cv::Mat RLE_to_DCT(const std::vector<std::pair<float, int>>& rleStream, int width, int height) {
    cv::Mat dctImage = cv::Mat::zeros(height, width, CV_32F);
    
    int i = 0, j = 0;
    int direction = 1; // 1 para arriba, -1 para abajo

    // Recorrer la secuencia RLE y reconstruir la matriz DCT
    for (int k = 0; k < rleStream.size(); ++k) {
        auto value = rleStream[k].first;
        int count = rleStream[k].second;

        while (count > 0) {
            // Asignar el valor actual de RLE al elemento correspondiente en la matriz DCT
            dctImage.at<float>(i, j) = value;

            // Moverse en zigzag
            if (direction == 1) {
                if (j == width - 1) { i++, direction = -1; } // Cambio de dirección en la última columna
                else if (i == 0) { j++, direction = -1; } // Cambio de dirección en la primera fila
                else { i--, j++; }
            } else {
                if (i == height - 1) { j++, direction = 1; } // Cambio de dirección en la última fila
                else if (j == 0) { i++, direction = 1; } // Cambio de dirección en la primera columna
                else { i++, j--; }
            }

            count--;
        }
    }

    return dctImage;
}

std::shared_ptr<coded_interfaces::msg::Rleimg>to_code_frame(const sensor_msgs::msg::Image::SharedPtr& frame , float compression_k) {
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

    auto rle_msg_ptr = std::make_shared<coded_interfaces::msg::Rleimg>(DCT_to_RLE(mat));


    return rle_msg_ptr;
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




