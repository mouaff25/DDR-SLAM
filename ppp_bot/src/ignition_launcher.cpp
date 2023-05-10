#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IgnitionNode : public rclcpp::Node
{
  public:
    IgnitionNode()
    : Node("ignition_launcher")
    {
        declare_parameter("world_file", "empty.sdf");
        get_parameter("world_file", world_file_);
        launch_ignition();
    }

  private:
    void launch_ignition()
    {
      RCLCPP_INFO(this->get_logger(), "Launching '%s' in Ignition Gazebo...", world_file_.c_str());
      std::string command = "ign gazebo " + world_file_;
      system(command.c_str());
      RCLCPP_INFO(this->get_logger(), "Ignition Gazebo closed.");
    }

    std::string world_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ignition_node = std::make_shared<IgnitionNode>();
  rclcpp::shutdown();
  return 0;
}