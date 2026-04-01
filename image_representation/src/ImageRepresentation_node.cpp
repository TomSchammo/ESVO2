#include <image_representation/ImageRepresentation.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<image_representation::ImageRepresentation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
