#include "advanced_depth_codec/advanced_depth_codec.hpp"  // Asegúrate de que el include sea correcto
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/rleimg.hpp"




std::vector<double> DCT_2_zigzag(const cv::Mat& dctImage) {



    const int max_width = dctImage.cols; //1280
    const int max_height = dctImage.rows;  //720
    std::vector<double> zigzag_array;
    
    int n = 0; // Filas
    int m = 0; // Columnas
    std::string dir = "pos";

    while (dir != "stop") {

        zigzag_array.push_back(dctImage.at<float>(n, m));
        if (dir == "pos") {
            if (m < max_width - 1) {
                m += 1;
                if (n == 0) {
                    dir = "neg";
                } else {
                    n -= 1;
                }
            } else {
                n += 1;
                dir = "neg";
            }
        } else if (dir == "neg") {
            if (n < max_height - 1) {
                n += 1;
                if (m == 0) {
                    dir = "pos";
                } else {
                    m -= 1;
                }
            } else {
                m += 1;
                dir = "pos";
            }
        }
        if (m == max_width - 1 && n == max_height - 1) {
            dir = "stop";
            zigzag_array.push_back(dctImage.at<float>(n, m));
        }
    }

    return zigzag_array;
}

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
    //cv::Mat processedImage;
    //cv::idct(dctImage, processedImage);

    // Convert the processed image back to mono16
    //cv::Mat finalImage;
    //processedImage.convertTo(finalImage, CV_16U);

    return dctImage;
}

coded_interfaces::msg::Rleimg DCT_to_RLE(const cv::Mat& dctImage) {
    // Esta función asume que dctImage es una matriz cuadrada

    std::vector<float> rle_values;
    std::vector<int> rle_counts;

    // Recorrido en zigzag
    auto zigzagArray = DCT_2_zigzag(dctImage);

    double currentVal = zigzagArray[0];
    int count = 1;

    for (size_t i = 1; i < zigzagArray.size(); ++i) {
        if (zigzagArray[i] == currentVal) {
            // Si el valor actual es igual al último, incrementa el contador.
            ++count;
        } else {
            // Si encontramos un valor diferente, guardamos el valor actual y su contador,
            // y luego reiniciamos el contador para el nuevo valor.
            rle_values.push_back(currentVal);
            rle_counts.push_back(count);
            currentVal = zigzagArray[i];
            count = 1;
        }
    }


    // No olvides agregar el último valor y su contador.
    rle_values.push_back(currentVal);
    rle_counts.push_back(count);

    // Construir el mensaje RLEImage con los vectores y dimensiones originales
    coded_interfaces::msg::Rleimg rle_image_msg;
    rle_image_msg.original_width = dctImage.cols;
    rle_image_msg.original_height = dctImage.rows; // O utiliza la altura real si la imagen no es cuadrada
    rle_image_msg.rle_values = rle_values;
    rle_image_msg.rle_counts = rle_counts;



    return rle_image_msg;
}

cv::Mat RLE_to_DCT(const std::vector<std::pair<float, int>>& rleStream, int width, int height) {
    cv::Mat dctImage = cv::Mat::zeros(height, width, CV_32F);
    
    int i = 0, j = 0;
    int direction = 1; // 1 para arriba, -1 para abajo

    // Recorrer la secuencia RLE y reconstruir la matriz DCT
    for (size_t k = 0; k < rleStream.size(); ++k) {
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

    cv::Mat &mat = cv_ptr->image;
    
    mat = DCT_coding(mat, compression_k);
    auto rle_msg_ptr = std::make_shared<coded_interfaces::msg::Rleimg>(DCT_to_RLE(mat));

    return rle_msg_ptr;
}


std::shared_ptr<sensor_msgs::msg::Image> to_decode_frame(const coded_interfaces::msg::Rleimg::SharedPtr& msg) {
    int width = msg->original_width;
    int height = msg->original_height;

    std::vector<std::pair<float, int>> rleStream;

    // Llena el vector combinado
    for (size_t i = 0; i < msg->rle_values.size(); ++i) {
        rleStream.push_back(std::make_pair(msg->rle_values[i], msg->rle_counts[i]));
    }

    // Nota rle_stream es de tipo  std::vector<std::pair<float, int>>& rleStream,
    auto dct_reconstructed = RLE_to_DCT(rleStream ,width,height);

    cv::Mat processedImage;
    cv::idct(dct_reconstructed, processedImage); // Asegúrate de que `dct_coefficients` está correctamente inicializada y llena.

    // Convertir la imagen procesada a mono16
    cv::Mat finalImage;
    processedImage.convertTo(finalImage, CV_16U);

    auto processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", finalImage).toImageMsg();

    return processed_image_msg;
}



