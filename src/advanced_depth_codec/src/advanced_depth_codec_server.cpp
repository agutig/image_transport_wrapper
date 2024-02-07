#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "advanced_depth_codec/advanced_depth_codec.hpp"
#include "advanced_depth_codec/msg/Rleimg.hpp"

const std::string TOPIC_IN = "depth_camera_image";
const std::string TOPIC_OUT = "depth_server_camera_image";

class advanced_depth_codec_server : public rclcpp::Node
{
public:
  advanced_depth_codec_server() : Node("advanced_depth_codec_server")
  {
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", TOPIC_IN.c_str());
    RCLCPP_INFO(this->get_logger(), "Ready to publish on: %s", TOPIC_OUT.c_str());

    this -> declare_parameter<float>("compression_k", 1.2); //Make this value bigger for higher compression

    // Define the QoS profile
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(1);

    publisher_ = this->create_publisher<advanced_depth_codec::msg::RLEimg>(TOPIC_OUT, qos);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      TOPIC_IN, qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {

        RCLCPP_INFO(this->get_logger(), "Connected to topics, sending video...");
        float compression_k = 1;
        this -> get_parameter("compression_k", compression_k);
        auto codec_msg = to_code_frame(msg ,compression_k) ;

        publisher_->publish(*codec_msg);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<advanced_depth_codec_server>());
  rclcpp::shutdown();
  return 0;
}