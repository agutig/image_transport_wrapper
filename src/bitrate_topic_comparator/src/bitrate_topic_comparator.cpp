#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

using namespace std::chrono_literals;

const std::string TOPIC_1 = "depth_camera_image";

class BitrateTopicComparator : public rclcpp::Node {
public:
    BitrateTopicComparator()
    : Node("bitrate_topic_comparator"), last_bitrate_(-1000.0), last_message_time_(this->now()) {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos.keep_last(1);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            TOPIC_1, qos, std::bind(&BitrateTopicComparator::topic_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            1s, std::bind(&BitrateTopicComparator::timer_callback, this));
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        //RCLCPP_INFO(this->get_logger(), "Mensaje recibido en topic '%s'", TOPIC_1.c_str());
        auto now = this->now();
        auto time_diff = now - last_message_time_;
        auto time_diff_sec = time_diff.seconds() + time_diff.nanoseconds() / 1e9;

        if (time_diff_sec > 0) {
            size_t message_size_bits = calculate_serialized_size(msg);
            last_bitrate_ = message_size_bits / time_diff_sec;
            last_bitrate_ = last_bitrate_ / 1e6;
        }

        last_message_time_ = now;
    }

    void timer_callback() {
        if (last_bitrate_ <= 0) {
            RCLCPP_INFO(this->get_logger(), "%s -- NO data", TOPIC_1.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "%s -- Bitrate: %f Mbits/sec", TOPIC_1.c_str(), last_bitrate_);
        }
    }

    size_t calculate_serialized_size(const sensor_msgs::msg::Image::SharedPtr& msg) {
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(msg.get(), &serialized_msg);
        size_t serialized_size_bytes = serialized_msg.size();
        return serialized_size_bytes * 8;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double last_bitrate_;
    rclcpp::Time last_message_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BitrateTopicComparator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

