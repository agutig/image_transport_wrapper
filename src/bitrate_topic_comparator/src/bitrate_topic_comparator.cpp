#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

using namespace std::chrono_literals;

const std::string TOPIC_1 = "depth_camera_image";
const std::string TOPIC_2 = "depth_server_camera_image"; // Definimos el segundo topic

class BitrateTopicComparator : public rclcpp::Node {
public:
    BitrateTopicComparator()
    : Node("bitrate_topic_comparator"),
      last_bitrate_1_(-1000.0), last_message_time_1_(this->now()),
      last_bitrate_2_(-1000.0), last_message_time_2_(this->now()) { // Inicializamos las variables para ambos topics
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos.keep_last(1);

        // Suscripción al primer topic
        subscription_1_ = this->create_subscription<sensor_msgs::msg::Image>(
            TOPIC_1, qos, std::bind(&BitrateTopicComparator::topic_callback_1, this, std::placeholders::_1));
            
        // Suscripción al segundo topic
        subscription_2_ = this->create_subscription<sensor_msgs::msg::Image>(
            TOPIC_2, qos, std::bind(&BitrateTopicComparator::topic_callback_2, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            1s, std::bind(&BitrateTopicComparator::timer_callback, this));
    }

private:
    // Callback para el primer topic
    void topic_callback_1(const sensor_msgs::msg::Image::SharedPtr msg) {
        calculate_bitrate(msg, last_bitrate_1_, last_message_time_1_);
    }
    
    // Callback para el segundo topic
    void topic_callback_2(const sensor_msgs::msg::Image::SharedPtr msg) {
        calculate_bitrate(msg, last_bitrate_2_, last_message_time_2_);
    }

    // Función para calcular el bitrate
    void calculate_bitrate(const sensor_msgs::msg::Image::SharedPtr& msg, double& last_bitrate, rclcpp::Time& last_message_time) {
        auto now = this->now();
        auto time_diff = now - last_message_time;
        auto time_diff_sec = time_diff.seconds() + time_diff.nanoseconds() / 1e9;

        if (time_diff_sec > 0) {
            size_t message_size_bits = calculate_serialized_size(msg);
            last_bitrate = message_size_bits / time_diff_sec / 1e6; // Mbits/sec
        }

        last_message_time = now;
    }

    // Timer callback para imprimir los bitrates
    void timer_callback() {
      std::stringstream ss;
      // Para el primer topic
      if (last_bitrate_1_ <= 0) {
          ss << TOPIC_1 << " Bitrate: No data";
      } else {
          ss << TOPIC_1 << " Bitrate: " << last_bitrate_1_ << " Mbits/sec";
      }
      // Añade una coma y espacio para separar los mensajes de los topics
      ss << ", ";
      // Para el segundo topic
      if (last_bitrate_2_ <= 0) {
          ss << TOPIC_2 << " Bitrate: No data";
      } else {
          ss << TOPIC_2 << " Bitrate: " << last_bitrate_2_ << " Mbits/sec";
      }

      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

      last_bitrate_1_ = -1000;
      last_bitrate_2_ = -1000;
    }

    // Función para calcular el tamaño serializado del mensaje
    size_t calculate_serialized_size(const sensor_msgs::msg::Image::SharedPtr& msg) {
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(msg.get(), &serialized_msg);
        return serialized_msg.size() * 8; // bits
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_1_, subscription_2_;
    rclcpp::TimerBase::SharedPtr timer_;
    double last_bitrate_1_, last_bitrate_2_;
    rclcpp::Time last_message_time_1_, last_message_time_2_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BitrateTopicComparator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
