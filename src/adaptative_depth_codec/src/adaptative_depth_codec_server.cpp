#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "adaptative_depth_codec/adaptative_depth_codec.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"
#include "coded_interfaces/msg/rleimg.hpp"
#include "coded_interfaces/msg/adaptative.hpp"
#include <tuple>
#include <chrono>

const std::string TOPIC_IN = "depth_camera_image"; //Input topic
const std::string TOPIC_OUT = "depth_server_camera_image";  //Output topic
const std::string ADAPTATIVE_TOPIC = "depth_adaptative_channel";


bool handshake_done = false;

class adaptative_depth_codec_server : public rclcpp::Node
{

//This node calls a codec server for depth raw video (16 bits) and codecs it into an rleimg message (look at coded interfaces pkg).
//This is rleimg message:
//
//  * uint32 original_width
//  * uint32 original_height
//  * float32[] rle_values #values 
//  * int32[] rle_counts  #number of times each value repeats


//K values and bitrate

// K = 1 --> 18 Mbits max
// K = 2 --> 16 Mbits max
// K = 3 --> 12 Mbits max
// K = 4 --> 8 Mbits max
// K = 5 --> 6 Mbits max
// K = 10 --> 1.8 Mbits max
// K = 15 --> 0.8
// K = 20 --> 0.5
// K = 25 --> 0.3
// K = 25 --> 0.8

public:
  adaptative_depth_codec_server() : Node("adaptative_depth_codec_server")
  {
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", TOPIC_IN.c_str());
    RCLCPP_INFO(this->get_logger(), "Ready to publish on: %s", TOPIC_OUT.c_str());

    this -> declare_parameter<float>("compression_k", 2); //This param can be managed from outside.Make this value bigger for higher compression.
    // If k<2 i can be bigger than the original data

    // Define the QoS profile
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(1);

    publisher_ = this->create_publisher<coded_interfaces::msg::Rleimg>(TOPIC_OUT, qos);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      TOPIC_IN, qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {

        RCLCPP_INFO(this->get_logger(), "Connected to topics, sending video...");

        float compression_k = 1;
        this -> get_parameter("compression_k", compression_k);
        auto codec_msg = to_code_frame(msg ,compression_k) ;
        publisher_->publish(*codec_msg);

      });

    // Adaptative topic
    publisher_adaptative_ = this->create_publisher<coded_interfaces::msg::Adaptative>(ADAPTATIVE_TOPIC, 10);
    subscription_adaptative_ = this->create_subscription<coded_interfaces::msg::Adaptative>(
            ADAPTATIVE_TOPIC, 10, std::bind(&adaptative_depth_codec_server::adaptative_topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer( std::chrono::seconds(1),std::bind(&adaptative_depth_codec_server::status_callback, this));
  }

private:

  void adaptative_topic_callback(const coded_interfaces::msg::Adaptative::SharedPtr msg)
    {
        // Imprime el contenido del mensaje recibido.
        RCLCPP_INFO(this->get_logger(), "Message on adaptative topic recived");
        RCLCPP_INFO(this->get_logger(), "Recibido: role='%s', msg_type=%u, msg_json='%s'",
                    msg->role.c_str(), msg->msg_type, msg->msg_json.c_str());


        if (msg->role == "client"){
          if (msg->msg_type == 0){
            auto result = select_k(msg->msg_json);

            float update_k_value = std::stof(std::get<0>(result)); 
            coded_interfaces::msg::Adaptative answer_msg = std::get<1>(result);
            handshake_done = std::get<2>(result);
            
            publisher_adaptative_->publish(answer_msg);
            this->set_parameter(rclcpp::Parameter("compression_k", update_k_value));
            RCLCPP_INFO(this->get_logger(), "Codec configured, k value: %f", update_k_value);

          }
        }
        
    }

  void status_callback(){

    //This function will send status messages to the client
    if (handshake_done) {
        RCLCPP_INFO(this->get_logger(), "Hello worrd");
        // Publicar el mensaje
        //publisher_adaptative_->publish(message);
    }
  }


  rclcpp::Publisher<coded_interfaces::msg::Rleimg>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;


  rclcpp::Publisher<coded_interfaces::msg::Adaptative>::SharedPtr publisher_adaptative_;
  rclcpp::Subscription<coded_interfaces::msg::Adaptative>::SharedPtr subscription_adaptative_;

  rclcpp::TimerBase::SharedPtr timer_ ;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adaptative_depth_codec_server>());
  rclcpp::shutdown();
  return 0;
}