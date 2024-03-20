#include <memory> // Para std::shared_ptr
#include "coded_interfaces/msg/adaptative.hpp"
#include "nlohmann/json.hpp"
#include <tuple>


coded_interfaces::msg::Adaptative generate_client_handshake(int fps=0, int height=0, int weight=0, int frame_type=0, int max_bit_rate=0);
coded_interfaces::msg::Adaptative generate_server_handshake(int fps=0, int height=0, int weight=0, int frame_type=0, int max_bit_rate=0);
coded_interfaces::msg::Adaptative generate_server_status(int fps=0, int height=0, int weight=0, int frame_type=0, int max_bit_rate=0);

std::tuple<std::string, coded_interfaces::msg::Adaptative, bool> select_k(std::string& msg_json);