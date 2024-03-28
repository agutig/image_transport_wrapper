#ifndef ADAPTIVE_ALG_HPP
#define ADAPTIVE_ALG_HPP

#include <memory> // Para std::shared_ptr
#include "coded_interfaces/msg/adaptative.hpp"
#include "nlohmann/json.hpp"
#include <tuple>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/rleimg.hpp"
#include <nlohmann/json.hpp>

coded_interfaces::msg::Adaptative generate_client_handshake(int fps=0, int height=0, int weight=0, int frame_type=0, double max_bit_rate=0);
coded_interfaces::msg::Adaptative generate_server_handshake(bool accepted, const std::string& error_msg ,  double procesing_time);
coded_interfaces::msg::Adaptative generate_server_status(int fps=0, int height=0, int weight=0, int frame_type=0, double max_bit_rate=0 , double procesing_time=0.1);
coded_interfaces::msg::Adaptative generate_client_status(int fps=0, int height=0, int weight=0, int frame_type=0, double max_bit_rate=0);

std::tuple<std::string, bool> select_k(std::string& msg_json, std::string compression_k,nlohmann::json codecConfigs);
std::tuple<std::string, bool> select_k_2(std::string& msg_json, std::string compression_k);

//

class BitrateCalculator {
public:
    rclcpp::Clock::SharedPtr clock_; // Usar un shared_ptr para el reloj
    rclcpp::Time last_message_time;  // Ya no necesita inicialización por defecto aquí
    int frame_rate;
    double processing_time;    
    std::vector<float> latency_buffer;
    std::vector<float> bitrate_buffer;

    // Constructor que acepta un reloj
    BitrateCalculator()
        : clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), 
        last_message_time(clock_->now()) {}

    double calculate_bitrate_to_request(); // Método modificado para devolver el tiempo
    void add_msg_data_to_buffer(const std::shared_ptr<coded_interfaces::msg::Rleimg>& msg); 
};

#endif // BITRATE_CALCULATOR_HPP