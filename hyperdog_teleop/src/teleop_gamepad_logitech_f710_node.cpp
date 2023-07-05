#include <cstdio>
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "hyperdogv2_msgs/msg/joy_ctrl_cmds.hpp"


using namespace std::chrono_literals;

auto teleop_cmds_ = hyperdogv2_msgs::msg::JoyCtrlCmds();
auto joy_fb = sensor_msgs::msg::JoyFeedback();
clock_t t1, t2, t3 = clock();
int btn_toggle_delay_ = 5000; // 0.5 sec


class HyperdogTeleopF710 : public rclcpp::Node
{
  private:
    
    rclcpp::TimerBase::SharedPtr params_update_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<hyperdogv2_msgs::msg::JoyCtrlCmds>::SharedPtr teleop_joy_publisher_;

    int max_height_, min_height_, height_increment_gain_, roll_limit_, pitch_limit_, yaw_limit_;
    int lean_x_min_, lean_x_max_, lean_y_min_, lean_y_max_;
    int max_step_length_x_, max_step_length_y_, min_step_height_, max_step_height_;

    void declare_parameters()
    {
      this->declare_parameter("min_height");
      this->declare_parameter("max_height");
      this->declare_parameter("height_increment_gain");
      this->declare_parameter("roll_limit");
      this->declare_parameter("pitch_limit");
      this->declare_parameter("yaw_limit");
      this->declare_parameter("lean_x_min");
      this->declare_parameter("lean_x_max");
      this->declare_parameter("lean_y_min");
      this->declare_parameter("lean_y_max");
      this->declare_parameter("max_step_length_x");
      this->declare_parameter("max_step_length_y");
      this->declare_parameter("min_step_height");
      this->declare_parameter("max_step_height");
    }

    void read_parameters()
    {
      this->get_parameter("min_height", min_height_);
      this->get_parameter("max_height", max_height_);
      this->get_parameter("height_increment_gain", height_increment_gain_);
      this->get_parameter("roll_limit", roll_limit_);
      this->get_parameter("pitch_limit", pitch_limit_);
      this->get_parameter("yaw_limit", yaw_limit_);
      this->get_parameter("lean_x_min", lean_x_min_);
      this->get_parameter("lean_x_max", lean_x_max_);
      this->get_parameter("lean_y_min", lean_y_min_);
      this->get_parameter("lean_y_max", lean_y_max_);
      this->get_parameter("max_step_length_x", max_step_length_x_);
      this->get_parameter("max_step_length_y", max_step_length_y_);
      this->get_parameter("min_step_height", min_step_height_);
      this->get_parameter("max_step_height", max_step_height_);
      // RCLCPP_INFO(this->get_logger(), "parameters are updated"); // --- debug ---
      // RCLCPP_INFO(this->get_logger(), "min_height: %i", min_height_); // -- debug --
      // RCLCPP_INFO(this->get_logger(), "max_height: %i", max_height_); // -- debug --
    }

    bool is_btn_toggle_ready(clock_t* t){
      if (clock() - *t > btn_toggle_delay_){
        *t = clock();
        return true;
      }
      else return false;
    }

    void joy_subscriber_callback(sensor_msgs::msg::Joy::SharedPtr msg)
    {
      // set robot start state from start btn
      if(!teleop_cmds_.start && msg->buttons[7] && is_btn_toggle_ready(&t1))
        teleop_cmds_.start = true;
      else if(teleop_cmds_.start && msg->buttons[7] && is_btn_toggle_ready(&t1))
        teleop_cmds_.start = false;

      // if robot is stated
      if(teleop_cmds_.start){
        // set robot walk state from back button
        if(!teleop_cmds_.walk && msg->buttons[6] && is_btn_toggle_ready(&t2))
          teleop_cmds_.walk = true;
        else if(teleop_cmds_.walk && msg->buttons[6] && is_btn_toggle_ready(&t2))
          teleop_cmds_.walk = false;

        // set side_move_mode state from right joystick button
        if(!teleop_cmds_.side_move_mode && msg->buttons[10] && is_btn_toggle_ready(&t3))
          teleop_cmds_.side_move_mode = true;
        else if(teleop_cmds_.side_move_mode && msg->buttons[10] && is_btn_toggle_ready(&t3)) 
          teleop_cmds_.side_move_mode = false;

        // select gait_mode from A, B, X, Y buttuns
        if(msg->buttons[0]) teleop_cmds_.gait_type = 0; // A button
        if(msg->buttons[1]) teleop_cmds_.gait_type = 1; // B button
        if(msg->buttons[2]) teleop_cmds_.gait_type = 2; // X button
        if(msg->buttons[3]) teleop_cmds_.gait_type = 3; // Y button

        // set robot height from hat_x buttons 
        if(msg->axes[7] != 0)
          teleop_cmds_.body_transform.translation.z += height_increment_gain_ * msg->axes[7];
        if(teleop_cmds_.body_transform.translation.z < (double)min_height_)
          teleop_cmds_.body_transform.translation.z = (double)min_height_;
        if(teleop_cmds_.body_transform.translation.z > (double)max_height_)
          teleop_cmds_.body_transform.translation.z = (double)max_height_;

        // set step height from hat_y buttons
        if(msg->axes[6] != 0)
          teleop_cmds_.gait_step.z += height_increment_gain_ * (- msg->axes[6]);
        if(teleop_cmds_.gait_step.z < min_step_height_) 
          teleop_cmds_.gait_step.z = min_step_height_; // set lower limit
        if(teleop_cmds_.gait_step.z > teleop_cmds_.body_transform.translation.z - min_height_) 
          teleop_cmds_.gait_step.z = teleop_cmds_.body_transform.translation.z - min_height_;  // set upper limit
        
        // set euler angles from left joystick and RT + LT : when not pressed LB button
        if(!msg->buttons[4]){
          teleop_cmds_.body_transform.rotation.x = -msg->axes[0]*roll_limit_;
          teleop_cmds_.body_transform.rotation.y = msg->axes[1]*pitch_limit_;
          teleop_cmds_.body_transform.rotation.z = ((msg->axes[2]+1)/2 - (msg->axes[5]+1)/2)*yaw_limit_;
        }

        // set lean [x, y] from left joystick when LB btn is pressed
        if(msg->buttons[4]){
          if(msg->axes[1]>0)
            teleop_cmds_.body_transform.translation.x = (double)(msg->axes[1]*lean_x_max_);
          else 
            teleop_cmds_.body_transform.translation.x = -msg->axes[1]*lean_x_min_;
          if(msg->axes[0]>0)
            teleop_cmds_.body_transform.translation.y = msg->axes[0]*lean_y_max_;
          else 
            teleop_cmds_.body_transform.translation.y = -msg->axes[0]*lean_y_min_;
        }

        // set gait step len [x, y]
        teleop_cmds_.gait_step.x = msg->axes[4]*max_step_length_x_;
        teleop_cmds_.gait_step.y = -msg->axes[3]*max_step_length_y_;
      }
      // if start state is not enable, disable walk mode
      else teleop_cmds_.walk = false;

      // set topic header stamp
      teleop_cmds_.header.stamp = this->get_clock()->now();
      // publish the topic
      teleop_joy_publisher_->publish(teleop_cmds_);
      
    }

  public:
    HyperdogTeleopF710()
    : Node("hyperdog_teleop_gamepad_node")
    {
      // Declare parameters
      this->declare_parameters();
      // set initial teleop cmds
      teleop_cmds_.body_transform.translation.z = (double)min_height_;
      teleop_cmds_.gait_step.z = (double)min_step_height_;
      // Read parameters
      this->read_parameters();
      // create a timer for updating parameters
      params_update_timer_ = this->create_wall_timer(1000ms, std::bind(&HyperdogTeleopF710::read_parameters, this));
      // create a subscriber to joy_node
      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&HyperdogTeleopF710::joy_subscriber_callback, this, std::placeholders::_1));
      teleop_joy_publisher_ = this->create_publisher<hyperdogv2_msgs::msg::JoyCtrlCmds>("hyperdog_joy_teleop_cmds", 10);
      
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
  printf("starting hyperdog_teleop package...\n");
  rclcpp::init(argc, argv);
  auto teleop_node = std::make_shared<HyperdogTeleopF710>();
  rclcpp::spin(teleop_node);
  rclcpp::shutdown();
  return 0;
}
