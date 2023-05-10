#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class URDFSpawnerNode : public rclcpp::Node
{
  public:
    URDFSpawnerNode()
    : Node("urdf_spawner")
    {
        declare_parameter("service", "/world/empty/create");
        declare_parameter("sdf_filename", "src/ppp_bot/description/robot.urdf");
        declare_parameter("sdf", "");
        declare_parameter("name", "urdf_model");

        get_parameter("service", service_);
        get_parameter("sdf_filename", sdf_filename_);
        get_parameter("sdf", sdf_);

        get_parameter("name", name_);

        spawn_urdf();
    }

  private:
    void spawn_urdf()
    {

      std::string command = "ign service -s " + service_;
      command += " --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean ";
      if (sdf_.empty())
      {
        RCLCPP_INFO(this->get_logger(), "Spawning '%s'...", sdf_filename_.c_str());
        command += "--timeout 1000 --req 'sdf_filename: \"" + sdf_filename_;
      }
      else
        command += "--timeout 1000 --req 'sdf: \"" + sdf_;
      command += "\", name: \"" + name_ + "\"";
      command += ", pose: {position: {z: 1.0}}'";

      system(command.c_str());
    }

    std::string service_;
    std::string sdf_filename_;
    std::string sdf_;
    std::string name_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ignition_node = std::make_shared<URDFSpawnerNode>();
  rclcpp::shutdown();
  return 0;
}