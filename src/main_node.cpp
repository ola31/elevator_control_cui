#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <string>
#include <memory>
#include <vector>

#include "elevator_control_cui/main_node.hpp"


MainNode::MainNode()
: rclcpp::Node("elevator_control_cui")
{
  RCLCPP_INFO(this->get_logger(), "elevator_control_cui");

}

MainNode::~MainNode()
{

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainNode>();

  rclcpp::WallRate loop_rate(10);

  usleep(1000000);  // 1 sec

  while (rclcpp::ok()) {

    rclcpp::spin_some(node);

    loop_rate.sleep();
  }
  // rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
