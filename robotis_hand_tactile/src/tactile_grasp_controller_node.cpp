#include "rclcpp/rclcpp.hpp"
#include "tactile_grasp_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotis_hand_tactile::TactileGraspController>());
  rclcpp::shutdown();
  return 0;
}

