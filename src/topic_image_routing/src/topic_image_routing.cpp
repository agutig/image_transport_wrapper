#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class topic_image_routing : public rclcpp::Node
{
public:
  topic_image_routing() : Node("topic_image_routing")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic_out", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "topic_in", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Recibiendo y reenviando imagen");
        publisher_->publish(*msg);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<topic_image_routing>());
  rclcpp::shutdown();
  return 0;
}