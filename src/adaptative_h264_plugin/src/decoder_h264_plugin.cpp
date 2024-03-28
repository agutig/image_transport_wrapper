#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace tu_paquete_ros2 {

class Decodificador : public rclcpp::Node {
public:
  Decodificador(const rclcpp::NodeOptions & options);

private:
  // Métodos y miembros para la decodificación
};

} // namespace tu_paquete_ros2