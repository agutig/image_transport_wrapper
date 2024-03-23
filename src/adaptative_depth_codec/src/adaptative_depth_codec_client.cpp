#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "adaptative_depth_codec/adaptative_depth_codec.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"
#include "coded_interfaces/msg/rleimg.hpp"
#include "coded_interfaces/msg/adaptative.hpp"


const std::string TOPIC_IN = "depth_server_camera_image";
const std::string TOPIC_OUT = "depth_client_camera_image";
const std::string ADAPTATIVE_TOPIC = "depth_adaptative_channel";

bool handshake_done = false;

class adaptative_depth_codec_client : public rclcpp::Node
{
//This node calls a codec client for an coded depth video (rleimg msg) and decodes it into an ordinary video (msg/image secuence).

public:
  BitrateCalculator bitrate_calculator;
  adaptative_depth_codec_client() : Node("adaptative_depth_codec_client")
  {
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", TOPIC_IN.c_str());
    RCLCPP_INFO(this->get_logger(), "Ready to publish on: %s", TOPIC_OUT.c_str());

    // Define the QoS profile
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(1);

    //define object
    
    
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(TOPIC_OUT, qos);
    subscription_ = this->create_subscription<coded_interfaces::msg::Rleimg>(
      TOPIC_IN, qos,
      [this](coded_interfaces::msg::Rleimg::SharedPtr msg) {

        auto now = this->now();
        RCLCPP_INFO(this->get_logger(), "Connected to topics, sending video...");
        auto decoded_msg = to_decode_frame(msg);

        publisher_->publish(*decoded_msg);
        
        double bitrate = this->bitrate_calculator.calculate_bitrate_to_request(msg,now);
        std::cout << "Bitrate solicitado: " << bitrate << " Mbits/sec" << std::endl;

      });

    
    publisher_adaptative_ = this->create_publisher<coded_interfaces::msg::Adaptative>(ADAPTATIVE_TOPIC, 10);
    timer_ = this->create_wall_timer( std::chrono::seconds(1),std::bind(&adaptative_depth_codec_client::comunicate_w_server, this));

    subscription_adaptative_ = this->create_subscription<coded_interfaces::msg::Adaptative>(
            ADAPTATIVE_TOPIC, 10, std::bind(&adaptative_depth_codec_client::adaptative_topic_callback, this, std::placeholders::_1));
  }

private:

  void adaptative_topic_callback(const coded_interfaces::msg::Adaptative::SharedPtr msg)
    {

        if (msg->role == "server"){
          RCLCPP_INFO(this->get_logger(), "Message on adaptative topic recived");
          RCLCPP_INFO(this->get_logger(), "Recibido: role='%s', msg_type=%u, msg_json='%s'",
          msg->role.c_str(), msg->msg_type, msg->msg_json.c_str());
          
          if (msg->msg_type == 0){

            RCLCPP_WARN(this->get_logger(), "Received Handshake: %d", msg->msg_type);
            handshake_done = true;

          } 

        }
    }

  void comunicate_w_server()
    {
      if (handshake_done){
          RCLCPP_INFO(this->get_logger(), "Handhsake done");
        
      }else{
          // Adaptative topic
          RCLCPP_INFO(this->get_logger(), "Sending handshake");
          publisher_adaptative_->publish(generate_client_handshake(0,0,0,0,0));
          RCLCPP_INFO(this->get_logger(), "Sent handshake");
      };
    };

  


  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<coded_interfaces::msg::Rleimg>::SharedPtr subscription_;

  rclcpp::Publisher<coded_interfaces::msg::Adaptative>::SharedPtr publisher_adaptative_;
  rclcpp::Subscription<coded_interfaces::msg::Adaptative>::SharedPtr subscription_adaptative_;

  rclcpp::TimerBase::SharedPtr timer_ ;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adaptative_depth_codec_client>());
  rclcpp::shutdown();
  return 0;
}