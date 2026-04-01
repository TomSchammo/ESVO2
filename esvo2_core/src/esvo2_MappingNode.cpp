#include <esvo2_core/esvo2_Mapping.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<esvo2_core::esvo2_Mapping>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
