#include <nlohmann/json.hpp>
#include <fstream> // Asegúrate de incluir esta cabecera
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/adaptative.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"
#include <tuple>
#include "coded_interfaces/msg/rleimg.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"


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

coded_interfaces::msg::Adaptative generate_server_status(int fps, int height, int width, int frame_type, int max_bit_rate) {
    auto message = coded_interfaces::msg::Adaptative();
    message.role = "server"; // Asumiendo que el rol es necesario como en el mensaje anterior
    message.msg_type = 1; // Ajusta según tu definición de msg_type para este tipo de mensaje

    nlohmann::json j;
    j["fps"] = fps;
    j["height"] = height;
    j["width"] = width;
    j["frame_type"] = frame_type; // Asume valores "progressive" o "interlaced"
    j["max_bit_rate"] = max_bit_rate;

    message.msg_json = j.dump(); // Serializa el objeto JSON a string

    return message;
}

coded_interfaces::msg::Adaptative generate_client_status(int fps, int height, int width, int frame_type, int max_bit_rate) {
    auto message = coded_interfaces::msg::Adaptative();
    message.role = "client"; // Asumiendo que el rol es necesario como en el mensaje anterior
    message.msg_type = 1; // Ajusta según tu definición de msg_type para este tipo de mensaje

    nlohmann::json j;
    j["fps"] = fps;
    j["height"] = height;
    j["width"] = width;
    j["frame_type"] = frame_type; // Asume valores "progressive" o "interlaced"
    j["max_bit_rate"] = max_bit_rate;

    message.msg_json = j.dump(); // Serializa el objeto JSON a string

    return message;
}



std::tuple<std::string, bool> select_k(std::string& msg_json) {

    using json = nlohmann::json;
    json root;

    std::cout << "msg_json='" << msg_json << "'" << std::endl;

    // Intenta parsear el JSON string a un objeto json
    try {
        root = json::parse(msg_json);
    } catch (json::parse_error& e) {
        std::cerr << "Error al parsear el JSON: " << e.what() << std::endl;
        return std::make_tuple("0" , false);
    }

    // Asume que max_bit_rate es un número y lo obtiene
    float searchValue = root["max_bit_rate"].get<float>();

   
    const std::string filename = "src/adaptative_depth_codec/src/utils/codec_configs.json";
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Error al abrir el archivo: " << filename << std::endl;
        return std::make_tuple("0" , false); // Retorna un string vacío en caso de error
    }

    // Parsea el archivo JSON
    json j;
    inputFile >> j;

    // Busca la clave para el valor dado
    for (const auto& item : j.items()) {
        // Asumiendo que estás buscando dentro de un objeto o array de configuraciones,
        // y cada elemento tiene una clave "max_bit_rate" que quieres comparar.
        if (item.value().contains("max_bit_rate") && item.value()["max_bit_rate"].get<float>() == searchValue) {
            return std::make_tuple(item.key(),true); // Retorna la clave si encuentra el valor
        }
    }

    return std::make_tuple("0", true);
}



double BitrateCalculator::calculate_bitrate_to_request(const std::shared_ptr<coded_interfaces::msg::Rleimg>& msg)
    {

        rclcpp::Serialization<coded_interfaces::msg::Rleimg> serialization;
        rclcpp::SerializedMessage serialized_msg;
        serialization.serialize_message(&(*msg), &serialized_msg);

        rclcpp::Time now = clock_->now();
        size_t message_size_bits = serialized_msg.size() * 8;
        auto time_diff = now - last_message_time;
        double time_diff_sec = time_diff.seconds();

        double actual_bitrate = 0.0;

        if (time_diff_sec > 0) {
            actual_bitrate = static_cast<double>(message_size_bits) / time_diff_sec / 1e6; // Mbits/sec
        }
        last_message_time = now;

        double extra_permited_latency = 0.01;
        double epsilon_margin = 0.2; //20%
        double media_time = extra_permited_latency + (1.0/ frame_rate) ;// Media real time, a time for a frame equals 1/frame rate
        double relative_capacity = time_diff_sec / media_time;
        double k = (1-epsilon_margin) * relative_capacity;

        double request_bitrate = k * actual_bitrate;
        std::cout << "________________"<< std::endl;
        std::cout << "Bitrate medido: " << actual_bitrate << " Mbits/sec" << std::endl;

        std::cout << "framerate: " << frame_rate << std::endl;
        std::cout << "media_time: " << media_time << std::endl;
        std::cout << "Latency: " << time_diff_sec << std::endl;

        std::cout << "relative_capacity: " << relative_capacity << std::endl;

        std::cout << "k: " << k << std::endl;

        std::cout << "Requested bitrate: " << request_bitrate << " Mbits/sec" << std::endl;

        return request_bitrate;
    }
