#include "sensor_msgs/msg/image.hpp"
#include <memory> // Para std::shared_ptr

std::shared_ptr<sensor_msgs::msg::Image> to_code_frame(const sensor_msgs::msg::Image::SharedPtr& frame , float compression_k);
std::shared_ptr<sensor_msgs::msg::Image> to_decode_frame(const sensor_msgs::msg::Image::SharedPtr& frame);
