#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

const int FRAMES_PER_SECOND = 60;


class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher"), cap(0) // Inicializa la cámara aquí
    {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara");
            rclcpp::shutdown(); // Si no se puede abrir la cámara, finaliza el nodo
        }

        // Define the QoS profile
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos.keep_last(1);

        publisher_ = image_transport::create_publisher(this, "camera_image", qos.get_rmw_qos_profile());
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / FRAMES_PER_SECOND),
            std::bind(&CameraPublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        cv::Mat frame;
        cap >> frame; // Captura un nuevo frame
        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_.publish(*msg);
        }
    }

    cv::VideoCapture cap;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
