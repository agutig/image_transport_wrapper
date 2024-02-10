#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "advanced_depth_codec/advanced_depth_codec.hpp"
#include "coded_interfaces/msg/rleimg.hpp"

const std::string TOPIC_IN = "depth_server_camera_image";
const std::string TOPIC_OUT = "depth_client_camera_image";

class advanced_depth_codec_client : public rclcpp::Node
{
//This node calls a codec client for an coded depth video (rleimg msg) and decodes it into an ordinary video (msg/image secuence).

public:
  advanced_depth_codec_client() : Node("advanced_depth_codec_client")
  {
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", TOPIC_IN.c_str());
    RCLCPP_INFO(this->get_logger(), "Ready to publish on: %s", TOPIC_OUT.c_str());

    // Define the QoS profile
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(1);

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(TOPIC_OUT, qos);
    subscription_ = this->create_subscription<coded_interfaces::msg::Rleimg>(
      TOPIC_IN, qos,
      [this](coded_interfaces::msg::Rleimg::SharedPtr msg) {

        RCLCPP_INFO(this->get_logger(), "Connected to topics, sending video...");
        auto decoded_msg = to_decode_frame(msg);

        publisher_->publish(*decoded_msg);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<coded_interfaces::msg::Rleimg>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<advanced_depth_codec_client>());
  rclcpp::shutdown();
  return 0;
}