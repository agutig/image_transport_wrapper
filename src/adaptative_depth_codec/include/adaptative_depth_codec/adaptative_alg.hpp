#include <memory> // Para std::shared_ptr
#include "coded_interfaces/msg/adaptative.hpp"
#include "nlohmann/json.hpp"


coded_interfaces::msg::Adaptative generate_client_handshake(int fps=0, int height=0, int weight=0, int frame_type=0, int max_bit_rate=0);