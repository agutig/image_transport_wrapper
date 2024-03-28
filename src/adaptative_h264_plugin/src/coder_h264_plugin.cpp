#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace adaptative_h264_plugin {

class h264_codec : public rclcpp::Node {
public:
  h264_codec(const rclcpp::NodeOptions & options);

private:
  // Métodos y miembros para la codificación
};

} // namespace tu_paquete_ros2
