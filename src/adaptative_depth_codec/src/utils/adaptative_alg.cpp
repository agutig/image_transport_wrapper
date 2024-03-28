#include <nlohmann/json.hpp>
#include <fstream> // Asegúrate de incluir esta cabecera
#include "rclcpp/rclcpp.hpp"
#include "coded_interfaces/msg/adaptative.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"
#include <tuple>
#include "coded_interfaces/msg/rleimg.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"


coded_interfaces::msg::Adaptative generate_client_handshake(int fps  , int height, int weight ,int frame_type,double max_bit_rate){
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

coded_interfaces::msg::Adaptative generate_server_handshake(bool accepted, const std::string& error_msg, double processing_time) {
    auto message = coded_interfaces::msg::Adaptative();
    message.role = "server";
    message.msg_type = 0; // Ajusta según tu definición de msg_type.

    nlohmann::json j;
    j["accepted"] = accepted; // Corregido el typo de "acepted" a "accepted"
    j["msg"] = error_msg; // Correctamente asignado a error_msg.
    j["processing_time"] = processing_time;

    std::string s = j.dump();
    message.msg_json = s;

    return message;
}

coded_interfaces::msg::Adaptative generate_server_status(int fps, int height, int width, int frame_type, double max_bit_rate, double processing_time) {
    auto message = coded_interfaces::msg::Adaptative();
    message.role = "server"; // Asumiendo que el rol es necesario como en el mensaje anterior
    message.msg_type = 1; // Ajusta según tu definición de msg_type para este tipo de mensaje

    nlohmann::json j;
    j["fps"] = fps;
    j["height"] = height;
    j["width"] = width;
    j["frame_type"] = frame_type; // Asume valores "progressive" o "interlaced"
    j["max_bit_rate"] = max_bit_rate;
    j["processing_time"] = processing_time;

    message.msg_json = j.dump(); // Serializa el objeto JSON a string

    return message;
}

coded_interfaces::msg::Adaptative generate_client_status(int fps, int height, int width, int frame_type, double max_bit_rate) {
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


std::string findNextKey(const nlohmann::json& codecConfigs, const std::string& currentKey) {
    int currentKeyInt = std::stoi(currentKey);
    std::string nextKey = "-1"; // Valor predeterminado que indica no encontrado
    int minDifference = std::numeric_limits<int>::max();
    
    for (auto& el : codecConfigs.items()) {
        int keyInt = std::stoi(el.key());
        if (keyInt > currentKeyInt && keyInt - currentKeyInt < minDifference) {
            nextKey = el.key();
            minDifference = keyInt - currentKeyInt;
        }
    }

    return nextKey;
}

// Función para encontrar la clave anterior
std::string findPreviousKey(const nlohmann::json& codecConfigs, const std::string& currentKey) {
    int currentKeyInt = std::stoi(currentKey);
    std::string previousKey = "-1"; // Valor predeterminado que indica no encontrado
    int minDifference = std::numeric_limits<int>::max();
    
    for (auto& el : codecConfigs.items()) {
        int keyInt = std::stoi(el.key());
        if (keyInt < currentKeyInt && abs(currentKeyInt- keyInt) < minDifference) {
            previousKey = el.key();
            minDifference = abs(currentKeyInt - keyInt);
        }
    }

    return previousKey;
}


std::tuple<std::string, bool> select_k(std::string& msg_json, std::string  actual_k, nlohmann::json codecConfigs) {

    using json = nlohmann::json;
    json root;

    //std::cout << "json: " << codecConfigs << "'" << std::endl;

    // Intenta parsear el JSON string a un objeto json
    try {
        root = json::parse(msg_json);
    } catch (json::parse_error& e) {
        std::cerr << "Error al parsear el JSON: " << e.what() << std::endl;
        return std::make_tuple("0" , false);
    }

    // Asume que max_bit_rate es un número y lo obtiene
    float searchValue = root["max_bit_rate"].get<float>();

    if (codecConfigs.find(actual_k) == codecConfigs.end()) {
        std::cerr << "actual_k not found" << std::endl;
        return std::make_tuple("0", false);
    }

    float value = codecConfigs[actual_k].get<float>();
    std::string posibleKey = actual_k; // Valor predeterminado
    std::string final_key = actual_k; // Valor predeterminado

    if (value > searchValue) {
        // Busca la siguiente clave mayor
        posibleKey = findNextKey(codecConfigs, actual_k);
        
        /*
        std::cout << " Disminuir Posible key: " << posibleKey << std::endl;
        std::cout << " Valor dic " << codecConfigs[posibleKey].get<float>() << std::endl;
        std::cout << " Actual key: " << actual_k << std::endl;
        std::cout << " Valor dic " << value << std::endl;
        std::cout << " Search value: " << searchValue << std::endl;
        */
        

        if (posibleKey != "-1"){
            final_key = posibleKey; 
        }
            
        /*if ( codecConfigs[posibleKey].get<float>() < searchValue ){
            // Si el bitrate deseado no supera al bitrate de la nueva key
            std::cout << "PAsa" << posibleKey << std::endl;
            final_key = actual_k;
        }*/
        return std::make_tuple(final_key, true);

    } else if (value < searchValue) {
        // Busca la clave anterior
        
        posibleKey = findPreviousKey(codecConfigs, actual_k);
        /*
        std::cout << " aumentar Posible key: " << posibleKey << std::endl;
        std::cout << " Posible Valor " << codecConfigs[posibleKey].get<float>() << std::endl;
        std::cout << " Actual key: " << actual_k << std::endl;
        std::cout << " Valor dic " << value << std::endl;
        std::cout << " Search value: " << searchValue << std::endl;
        */
        

        if (posibleKey != "-1"){
            final_key = posibleKey; 
        }

        if ( codecConfigs[posibleKey].get<float>() > searchValue ){
            // Si el bitrate deseado no supera al bitrate de la nueva key
            //std::cout << "Sw mantiene" << posibleKey << std::endl;
            final_key = actual_k;
        }
        return std::make_tuple(final_key, true);
        
    }   

    // Si resultKey sigue siendo actual_k, significa que no se encontró un cambio necesario o posible

    float new_target_rate = codecConfigs[actual_k].get<float>();

    return std::make_tuple(final_key, true);
}


std::tuple<std::string, bool> select_k_2(std::string& msg_json, std::string compression_k) {

    using json = nlohmann::json;
    json root;

    std::string evitar_warning_borrar = compression_k;
    //std::cout << "msg_json='" << msg_json << "'" << std::endl;

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
    float closestDiff = std::numeric_limits<float>::max(); // Inicializa con el máximo valor posible
    std::string closestKey;

    for (const auto& [key, value] : j.items()) {
        float currentValue = value.get<float>(); // Asumiendo que todos los valores son numéricos y pueden convertirse a float

        if (currentValue <= searchValue) {
            float currentDiff = std::fabs(currentValue - searchValue);
            if (currentDiff < closestDiff) {
                closestDiff = currentDiff;
                closestKey = key;
            }
        }
    }

    if (closestKey.empty()) {
        return std::make_tuple("0", true);// No se encontró ningún valor cercano
    }
    return std::make_tuple(closestKey, true); // Retorna la clave del valor más cercano

}



void add_to_buffer(std::vector<float>& buffer, float element, const size_t buffer_capacity) {
    if (buffer.size() >= buffer_capacity) {
        buffer.erase(buffer.begin()); // Elimina el primer elemento si el buffer está lleno
    }
    buffer.push_back(element); // Añade el nuevo elemento al final
}


float calculate_mean(const std::vector<float>& buffer) {
    if (buffer.empty()) return 0.0f; // Evita división por cero si el vector está vacío
    float suma = std::accumulate(buffer.begin(), buffer.end(), 0.0f);
    return suma / buffer.size();
}



double BitrateCalculator::calculate_bitrate_to_request()
    {

        double latency = calculate_mean(latency_buffer);
        double bitrate = bitrate_buffer.back();


        double extra_permited_latency = 0.001; //Latency asociated to procesing and channel.
        double epsilon_margin = 0.2; //20%

        double media_time = processing_time + extra_permited_latency + (1.0/ frame_rate);// Media real time, a time for a frame equals 1/frame rate
        double relative_capacity = media_time / latency;
        double k = (1-epsilon_margin) * relative_capacity;

        double request_bitrate = k * bitrate;
        std::cout << "________________"<< std::endl;
        std::cout << "Bitrate medido: " << bitrate << " Mbits/sec" << std::endl;

        std::cout << "framerate: " << frame_rate << std::endl;
        std::cout << "media_time: " << media_time << std::endl;
        std::cout << "Latency: " << latency << std::endl;
        std::cout << "Procesing time: " << processing_time << " sec" << std::endl;

        std::cout << "relative_capacity: " << relative_capacity << std::endl;

        std::cout << "coeficient: " << k << std::endl;

        std::cout << "Requested bitrate: " << request_bitrate << " Mbits/sec" << std::endl;

        return request_bitrate;
    }


void BitrateCalculator::add_msg_data_to_buffer(const std::shared_ptr<coded_interfaces::msg::Rleimg>& msg){
    

    rclcpp::Time timestamp_msg(msg->timestamp.sec, msg->timestamp.nanosec, RCL_ROS_TIME);

    rclcpp::Serialization<coded_interfaces::msg::Rleimg> serialization;
    rclcpp::SerializedMessage serialized_msg;
    serialization.serialize_message(&(*msg), &serialized_msg);

    rclcpp::Time now = clock_->now();
    size_t message_size_bits = serialized_msg.size() * 8;


    //auto time_diff = now - last_message_time; old
    auto time_diff = (now - timestamp_msg);
    double latency = time_diff.seconds();

    double actual_bitrate = 0.0;

    if (latency > 0) {
        actual_bitrate = static_cast<double>(message_size_bits) / latency / 1e6; // Mbits/sec
    }
    //std::cout << "Bitrate Estimado: " << static_cast<double>(message_size_bits)/ (1/frame_rate) / 1e6  << " Mbits/sec" << std::endl;
    last_message_time = now;

    add_to_buffer( latency_buffer,latency , frame_rate);
    add_to_buffer(bitrate_buffer,actual_bitrate,frame_rate);

}

