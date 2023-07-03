#include <cstdio>
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "hyperdogv2_msgs/msg/joy_ctrl_cmds.hpp"


using namespace std::chrono_literals;

auto cmds = hyperdogv2_msgs::msg::JoyCtrlCmds();


class HyperdogTeleopF710 : public rclcpp::Node
{
  private:
    
    rclcpp::TimerBase::SharedPtr params_update_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    int max_height_, min_height_, roll_limit_, pitch_limit_, yaw_limit_;
    int lean_x_min_, lean_x_max_, lean_y_min_, lean_y_max_;
    int max_step_length_x_, max_step_length_y_, max_step_height_;

    void declare_parameters()
    {
      this->declare_parameter("min_height");
      this->declare_parameter("max_height");
      this->declare_parameter("roll_limit");
      this->declare_parameter("pitch_limit");
      this->declare_parameter("yaw_limit");
      this->declare_parameter("lean_x_min");
      this->declare_parameter("lean_x_max");
      this->declare_parameter("lean_y_min");
      this->declare_parameter("lean_y_max");
      this->declare_parameter("max_step_length_x");
      this->declare_parameter("max_step_length_y");
      this->declare_parameter("max_step_height");
    }

    void read_parameters()
    {
      this->get_parameter("min_height", min_height_);
      this->get_parameter("max_height", max_height_);
      this->get_parameter("roll_limit", roll_limit_);
      this->get_parameter("pitch_limit", pitch_limit_);
      this->get_parameter("yaw_limit", yaw_limit_);
      this->get_parameter("lean_x_min", lean_x_min_);
      this->get_parameter("lean_x_max", lean_x_max_);
      this->get_parameter("lean_y_min", lean_y_min_);
      this->get_parameter("lean_y_max", lean_y_max_);
      this->get_parameter("max_step_length_x", max_step_length_x_);
      this->get_parameter("max_step_length_y", max_step_length_y_);
      this->get_parameter("max_step_height", max_step_height_);
      // RCLCPP_INFO(this->get_logger(), "parameters are updated"); // --- debug ---
      // RCLCPP_INFO(this->get_logger(), "min_height: %i", min_height_); // -- debug --
      // RCLCPP_INFO(this->get_logger(), "max_height: %i", max_height_); // -- debug --
    }

    void joy_subscriber_callback(sensor_msgs::msg::Joy::SharedPtr msg)
    {
      // set start btn
      // if (){

      // }

    }

  public:
    HyperdogTeleopF710()
    : Node("hyperdog_teleop_gamepad_node")
    {
      // Declare parameters
      this->declare_parameters();
      // Read parameters
      this->read_parameters();
      // create a timer for updating parameters
      params_update_timer_ = this->create_wall_timer(1000ms, std::bind(&HyperdogTeleopF710::read_parameters, this));
      // create a subscriber to joy_node
      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&HyperdogTeleopF710::joy_subscriber_callback, this, std::placeholders::_1));
    }

    // Destructor
    ~HyperdogTeleopF710(){
      RCLCPP_INFO(this->get_logger(), "closing teleop node");
    }


    

    

    

  
};


int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  printf("hello world hyperdog_teleop package\n");
  rclcpp::init(argc, argv);
  auto teleop_node = std::make_shared<HyperdogTeleopF710>();
  rclcpp::spin(teleop_node);
  rclcpp::shutdown();
  return 0;
}
