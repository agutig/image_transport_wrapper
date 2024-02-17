#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/adaptative.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"


void calculate_bitrate(size_t message_size_bits, double& last_bitrate, rclcpp::Time& last_message_time, rclcpp::Clock::SharedPtr clock) {
    auto now = clock->now();
    auto time_diff = now - last_message_time;
    // La conversión de la diferencia de tiempo a segundos se puede hacer directamente sin descomponer en segundos y nanosegundos.
    double time_diff_sec = time_diff.seconds();

    if (time_diff_sec > 0) {
        last_bitrate = message_size_bits / time_diff_sec / 1e6; // Mbits/sec
    }

    last_message_time = now;
}


coded_interfaces::msg::Adaptative generate_client_handshake(int fps  , int height, int weight ,int frame_type,int max_bit_rate){
    auto message = coded_interfaces::msg::Adaptative();
    message.role = "client";
    message.msg_type = 0; // Ajusta según tu definición de msg_type.
    
    nlohmann::json j;
    j["fps"] = fps; // Cambia a NaN o a otro valor según necesites.
    j["height"] = height; // Ejemplo para representar un NaN.
    j["weight"] = weight; // Ejemplo para representar un NaN.
    j["frame_type"] = frame_type; // Cambia a "interlaced" o a nullptr para NaN.
    j["max_bit_rate"] = max_bit_rate; // Ejemplo para representar un NaN.

    std::string s = j.dump(); 

    message.msg_json = s;

    return message;
}