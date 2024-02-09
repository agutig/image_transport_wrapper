#include "sensor_msgs/msg/image.hpp"
#include <memory> // Para std::shared_ptr
#include "coded_interfaces/msg/rleimg.hpp"

std::shared_ptr<coded_interfaces::msg::Rleimg> to_code_frame(const sensor_msgs::msg::Image::SharedPtr& frame , float compression_k);
std::shared_ptr<sensor_msgs::msg::Image> to_decode_frame(const coded_interfaces::msg::Rleimg::SharedPtr& msg);
