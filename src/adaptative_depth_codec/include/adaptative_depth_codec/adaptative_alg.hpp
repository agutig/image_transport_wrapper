#ifndef ADAPTIVE_ALG_HPP
#define ADAPTIVE_ALG_HPP

#include <memory> // Para std::shared_ptr
#include "coded_interfaces/msg/adaptative.hpp"
#include "nlohmann/json.hpp"
#include <tuple>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/rleimg.hpp"

coded_interfaces::msg::Adaptative generate_client_handshake(int fps=0, int height=0, int weight=0, int frame_type=0, int max_bit_rate=0);
coded_interfaces::msg::Adaptative generate_server_handshake(bool accepted, const std::string& error_msg);
coded_interfaces::msg::Adaptative generate_server_status(int fps=0, int height=0, int weight=0, int frame_type=0, int max_bit_rate=0);
coded_interfaces::msg::Adaptative generate_client_status(int fps=0, int height=0, int weight=0, int frame_type=0, int max_bit_rate=0);

std::tuple<std::string, bool> select_k(std::string& msg_json);

//

class BitrateCalculator {
public:
    rclcpp::Clock::SharedPtr clock_; // Usar un shared_ptr para el reloj
    rclcpp::Time last_message_time;  // Ya no necesita inicialización por defecto aquí
    int frame_rate; 

    // Constructor que acepta un reloj
    BitrateCalculator()
        : clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)), 
        last_message_time(clock_->now()) {}

    double calculate_bitrate_to_request(const std::shared_ptr<coded_interfaces::msg::Rleimg>& msg); // Método modificado para devolver el tiempo
};

#endif // BITRATE_CALCULATOR_HPP