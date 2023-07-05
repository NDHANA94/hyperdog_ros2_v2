#include <cstdio>
#include <chrono>
#include <functional>
#include <string>
#include <tgmath.h>
#include "rclcpp/rclcpp.hpp"
#include "hyperdogv2_msgs/msg/foot_positions.hpp"
#include "hyperdogv2_msgs/msg/joint_positions.hpp"
#include "geometry_msgs/msg/point32.hpp"


using namespace std::chrono_literals;


auto joint_positions_ = hyperdogv2_msgs::msg::JointPositions();
auto previous_joint_positions_ = hyperdogv2_msgs::msg::JointPositions();
auto foot_positions_ = hyperdogv2_msgs::msg::FootPositions();

class HyperdogIK : public rclcpp::Node
{
private:
    rclcpp::Subscription<hyperdogv2_msgs::msg::FootPositions>::SharedPtr foot_positions_subscriber_;
    rclcpp::Publisher<hyperdogv2_msgs::msg::JointPositions>::SharedPtr joint_positions_publisher_;

    int L1, L2, L3, L, W, MIN_ANGLE_L2L3, MAX_ANGLE_L2L3;
    int MIN_HEIGHT, MAX_HEIGHT, ROLL_LIMIT, PITCH_LIMIT, YAW_LIMIT;

    void declare_parameters()
    {
        this->declare_parameter("hip_link_len");
        this->declare_parameter("thigh_link_len");
        this->declare_parameter("calf_link_len");
        this->declare_parameter("dist_frontThighJointAxis2backThighJointAxis");
        this->declare_parameter("dist_leftHipJointAxis2rigthHipJointAxis");
        this->declare_parameter("min_angle_calf_joint");
        this->declare_parameter("max_angle_calf_link");
        this->declare_parameter("min_robot_height");
        this->declare_parameter("max_robot_height");
        this->declare_parameter("roll_limit");
        this->declare_parameter("pitch_limit");
        this->declare_parameter("yaw_limit");
    }

    void read_parameters()
    {
        this->get_parameter("hip_link_len", L1);
        this->get_parameter("thigh_link_len", L2);
        this->get_parameter("calf_link_len", L3);
        this->get_parameter("dist_frontThighJointAxis2backThighJointAxis", L);
        this->get_parameter("dist_leftHipJointAxis2rigthHipJointAxis", W);
        this->get_parameter("min_angle_calf_joint", MIN_ANGLE_L2L3);
        this->get_parameter("max_angle_calf_link", MAX_ANGLE_L2L3);
        this->get_parameter("min_robot_height", MIN_HEIGHT);
        this->get_parameter("max_robot_height", MAX_HEIGHT);
        this->get_parameter("roll_limit", ROLL_LIMIT);
        this->get_parameter("pitch_limit", PITCH_LIMIT);
        this->get_parameter("yaw_limit", YAW_LIMIT);
    }


    float* ik(geometry_msgs::msg::Point32 point)
    {
        float r_yz, alpha, th0, th1, th2;
        float*  joint_angles_ = new float[3];

        r_yz = sqrt(pow(point.y, 2) + pow(point.z, 2));
        alpha = acos(point.y/r_yz);
        th0 = acos(L1/r_yz) - alpha;
        // TODO



  
    }

    void publish_joint_angles(hyperdogv2_msgs::msg::FootPositions::SharedPtr msg)
    {
        
        float * fr_angles = ik(msg->fr);
        // TODO

    }

    

public:
    HyperdogIK()
    : Node("hyperdog_ik_node")
    {
        this->declare_parameters();
        this->read_parameters();
        foot_positions_subscriber_ = this->create_subscription<hyperdogv2_msgs::msg::FootPositions>\
                                    ("hyperdog_foot_positions", 10, std::bind(&HyperdogIK::publish_joint_angles, this, std::placeholders::_1));
        joint_positions_publisher_ = this->create_publisher<hyperdogv2_msgs::msg::JointPositions>("hyperdog_joint_positions", 10);
    }

    // Destructor
    ~HyperdogIK(){
        RCLCPP_INFO(this->get_logger(), "Closing hyperdog_ik node");
    }
};



int main(int argc, char ** argv)
{
  //(void) argc;
  //(void) argv;
  printf("starting ik node...\n");
  rclcpp::init(argc, argv);
  auto ik_node = std::make_shared<HyperdogIK>();
  rclcpp::spin(ik_node);
  rclcpp::shutdown();
  return 0;
}


