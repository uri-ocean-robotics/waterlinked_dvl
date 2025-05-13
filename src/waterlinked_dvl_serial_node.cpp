#include <rclcpp/rclcpp.hpp>
#include "waterlinked_dvl/waterlinked_dvl_serial.h" 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaterlinkedDvlSerial>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
