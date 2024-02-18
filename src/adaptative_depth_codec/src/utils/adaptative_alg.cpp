#include <nlohmann/json.hpp>
#include <fstream> // Asegúrate de incluir esta cabecera
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/adaptative.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"
#include <tuple>


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

coded_interfaces::msg::Adaptative generate_server_handshake(bool accepted, const std::string& error_msg) {
    auto message = coded_interfaces::msg::Adaptative();
    message.role = "server";
    message.msg_type = 0; // Ajusta según tu definición de msg_type.

    nlohmann::json j;
    j["accepted"] = accepted; // Corregido el typo de "acepted" a "accepted"
    j["msg"] = error_msg; // Correctamente asignado a error_msg.

    std::string s = j.dump();
    message.msg_json = s;

    return message;
}



std::tuple<std::string, coded_interfaces::msg::Adaptative, bool> select_k(std::string& msg_json) {

    using json = nlohmann::json;
    json root;

    std::cout << "msg_json='" << msg_json << "'" << std::endl;

    // Intenta parsear el JSON string a un objeto json
    try {
        root = json::parse(msg_json);
    } catch (json::parse_error& e) {
        std::cerr << "Error al parsear el JSON: " << e.what() << std::endl;
        return std::make_tuple("0" , generate_server_handshake(false, "Unable to read your handshake"), false);
    }

    // Asume que max_bit_rate es un número y lo obtiene
    float searchValue = root["max_bit_rate"].get<float>();

   
    const std::string filename = "src/adaptative_depth_codec/src/utils/codec_configs.json";
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Error al abrir el archivo: " << filename << std::endl;
        return std::make_tuple("0" , generate_server_handshake(false, "Error on the server side"),false); // Retorna un string vacío en caso de error
    }

    // Parsea el archivo JSON
    json j;
    inputFile >> j;

    // Busca la clave para el valor dado
    for (const auto& item : j.items()) {
        // Asumiendo que estás buscando dentro de un objeto o array de configuraciones,
        // y cada elemento tiene una clave "max_bit_rate" que quieres comparar.
        if (item.value().contains("max_bit_rate") && item.value()["max_bit_rate"].get<float>() == searchValue) {
            return std::make_tuple(item.key(), generate_server_handshake(true, ""),true); // Retorna la clave si encuentra el valor
        }
    }

    return std::make_tuple("0", generate_server_handshake(true, ""),true);
}


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


