#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "adaptative_depth_codec/adaptative_depth_codec.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"
#include "coded_interfaces/msg/rleimg.hpp"
#include "coded_interfaces/msg/adaptative.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

const std::string TOPIC_IN = "depth_server_camera_image";
const std::string TOPIC_OUT = "depth_client_camera_image";
const std::string ADAPTATIVE_TOPIC = "depth_adaptative_channel";

bool handshake_done = false;
bool reciving_video = false;

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

        reciving_video = true;
        //RCLCPP_INFO(this->get_logger(), "Connected to topics, sending video...");
        auto decoded_msg = to_decode_frame(msg);

        publisher_->publish(*decoded_msg);
        
        this->bitrate_calculator.add_msg_data_to_buffer(msg);
        //std::cout << "Bitrate solicitado: " << bitrate << " Mbits/sec" << std::endl
        

      });

    codec_timer_ = this->create_wall_timer( std::chrono::seconds(1),std::bind(&adaptative_depth_codec_client::comunicate_w_codec, this));

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

            try {
                auto json_obj = json::parse(msg->msg_json);
                std::cout << json_obj << std::endl;
                bitrate_calculator.processing_time = json_obj["processing_time"].get<double>();
                handshake_done = true;
            }catch (json::parse_error& e) {
                RCLCPP_ERROR(this->get_logger(), "Parsing error: %s", e.what());
            }

          } else if (msg->msg_type == 1)
          {
            try {
                auto json_obj = json::parse(msg->msg_json);
                std::cout << json_obj << std::endl;
                bitrate_calculator.processing_time = json_obj["processing_time"].get<double>();
            }catch (json::parse_error& e) {
                RCLCPP_ERROR(this->get_logger(), "Parsing error: %s", e.what());
            }
          }
          

        }
    }

  void comunicate_w_server()
    {
      if (!handshake_done){
          // Adaptative topic
          publisher_adaptative_->publish(generate_client_handshake(15,1080,1920,0,1000));
          bitrate_calculator.frame_rate = 15;
          
      };
    };

  void comunicate_w_codec()
    {
      if (handshake_done and reciving_video){
          double bitrate = this->bitrate_calculator.calculate_bitrate_to_request();
          publisher_adaptative_->publish(generate_client_status(15,1920,1080,0,bitrate));
        
      }
    };

  


  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<coded_interfaces::msg::Rleimg>::SharedPtr subscription_;

  rclcpp::Publisher<coded_interfaces::msg::Adaptative>::SharedPtr publisher_adaptative_;
  rclcpp::Subscription<coded_interfaces::msg::Adaptative>::SharedPtr subscription_adaptative_;

  rclcpp::TimerBase::SharedPtr timer_ ;
  rclcpp::TimerBase::SharedPtr codec_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adaptative_depth_codec_client>());
  rclcpp::shutdown();
  return 0;
}