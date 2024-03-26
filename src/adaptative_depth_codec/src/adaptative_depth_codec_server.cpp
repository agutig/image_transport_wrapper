#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "adaptative_depth_codec/adaptative_depth_codec.hpp"
#include "adaptative_depth_codec/adaptative_alg.hpp"
#include "coded_interfaces/msg/rleimg.hpp"
#include "coded_interfaces/msg/adaptative.hpp"
#include <tuple>
#include <chrono>
#include <map>

const std::string TOPIC_IN = "depth_camera_image"; //Input topic
const std::string TOPIC_OUT = "depth_server_camera_image";  //Output topic
const std::string ADAPTATIVE_TOPIC = "depth_adaptative_channel";



bool handshake_done = false;
std::string compression_k = "5";

double processing_video_time = 0.2;

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

std::map<int, double> bitrates = {
    {0, 1000000.0}, 
    {1, 9.0},    // K = 1 --> 18 Mbits max
    {2, 8.0},    // K = 2 --> 16 Mbits max
    {3, 6.0},    // K = 3 --> 12 Mbits max
    {4, 4.0},     // K = 4 --> 8 Mbits max
    {5, 3.0},     // K = 5 --> 6 Mbits max
    {10, 0.9},    // K = 10 --> 1.8 Mbits max
    {15, 0.4},    // K = 15 --> 0.8 Mbits max
    {20, 0.25},    // K = 20 --> 0.5 Mbits max
    {25, 0.15},    // K = 25 --> 0.3 Mbits max
};

public:
  adaptative_depth_codec_server() : Node("adaptative_depth_codec_server")
  {
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", TOPIC_IN.c_str());
    RCLCPP_INFO(this->get_logger(), "Ready to publish on: %s", TOPIC_OUT.c_str());

    this -> declare_parameter<std::string>("compression_k", compression_k); //This param can be managed from outside.Make this value bigger for higher compression.
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

        if (handshake_done){
          //
          //RCLCPP_INFO(this->get_logger(), "sending video...");
          auto start_time = std::chrono::high_resolution_clock::now();

          this -> get_parameter("compression_k", compression_k);
          auto codec_msg = to_code_frame(msg ,compression_k) ;
          publisher_->publish(*codec_msg);

          auto end_time = std::chrono::high_resolution_clock::now();
          processing_video_time = std::chrono::duration<double>(end_time - start_time).count();
          //std::cout << "Proceing time: " << processing_video_time << std::endl;

        }else {
          RCLCPP_INFO(this->get_logger(), "Reciving video...Waiting for handshake");
        }


      });

    // Adaptative topic
    publisher_adaptative_ = this->create_publisher<coded_interfaces::msg::Adaptative>(ADAPTATIVE_TOPIC, 10);
    subscription_adaptative_ = this->create_subscription<coded_interfaces::msg::Adaptative>(
            ADAPTATIVE_TOPIC, 10, std::bind(&adaptative_depth_codec_server::adaptative_topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer( std::chrono::seconds(10),std::bind(&adaptative_depth_codec_server::status_callback, this));
  }

private:

  void adaptative_topic_callback(const coded_interfaces::msg::Adaptative::SharedPtr msg)
    {
        // Imprime el contenido del mensaje recibido.
        std::cout << "" << std::endl;


        if (msg->role == "client"){
          if (msg->msg_type == 0){

            RCLCPP_WARN(this->get_logger(), "Received Handshake: %d", msg->msg_type);
            this -> get_parameter("compression_k", compression_k);
            auto result = select_k(msg->msg_json, compression_k);

            std::string update_k_value = std::get<0>(result); 
            //coded_interfaces::msg::Adaptative answer_msg = std::get<1>(result); //already returns a handshake msg
            handshake_done = std::get<1>(result);
            
            publisher_adaptative_->publish(generate_server_handshake(handshake_done, "", processing_video_time));
            this->set_parameter(rclcpp::Parameter("compression_k", update_k_value));
            RCLCPP_INFO(this->get_logger(), "Codec configured, k value: %s", update_k_value.c_str());

            RCLCPP_INFO(this->get_logger(), "Sending coded video on: %s", TOPIC_IN.c_str());

          } else if (msg->msg_type == 1) {

              RCLCPP_INFO(this->get_logger(), "Received a type 1 message.");
              this -> get_parameter("compression_k", compression_k);
              auto result = select_k(msg->msg_json,compression_k);
              std::string update_k_value = std::get<0>(result); 
              this->set_parameter(rclcpp::Parameter("compression_k", update_k_value));
              std::cout << "New profile K: " << update_k_value << std::endl;

              // Agrega aquí más lógica según sea necesario.
          } else {

            RCLCPP_WARN(this->get_logger(), "Received an unknown message type: %d", msg->msg_type);
          }
          
        }
        
    }

  void status_callback(){

    //This function will send status messages to the client
    if (handshake_done) {
        this -> get_parameter("compression_k", compression_k);
        auto msg =  generate_server_status( 15, 1920,1080, 0, 0, processing_video_time);
        publisher_adaptative_->publish(msg);

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