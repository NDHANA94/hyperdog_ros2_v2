#include <cstdio>
#include <chrono>
#include <functional>
#include <string>
#include <iostream>
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
    rclcpp::TimerBase::SharedPtr params_update_timer_;
    bool is_valid_fr, is_valid_fl, is_valid_br, is_valid_bl = false;

    int L1, L2, L3, L, W, MIN_ANGLE_L2L3, MAX_ANGLE_L2L3;
    int MIN_HEIGHT, MAX_HEIGHT, ROLL_LIMIT, PITCH_LIMIT, YAW_LIMIT;
    std::vector<long> fr_joint_dir, fl_joint_dir, br_joint_dir, bl_joint_dir;

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
        this->declare_parameter("fr_joint_directions");
        this->declare_parameter("fl_joint_directions");
        this->declare_parameter("br_joint_directions");
        this->declare_parameter("bl_joint_directions");

    }

    void read_parameters()
    {
        L1 = this->get_parameter("hip_link_len").as_int();
        L2 = this->get_parameter("thigh_link_len").as_int();
        L3 = this->get_parameter("calf_link_len").as_int();
        L = this->get_parameter("dist_frontThighJointAxis2backThighJointAxis").as_int();
        W = this->get_parameter("dist_leftHipJointAxis2rigthHipJointAxis").as_int();
        MIN_ANGLE_L2L3 = this->get_parameter("min_angle_calf_joint").as_int();
        MAX_ANGLE_L2L3 = this->get_parameter("max_angle_calf_link").as_int();
        MIN_HEIGHT = this->get_parameter("min_robot_height").as_int();
        MAX_HEIGHT = this->get_parameter("max_robot_height").as_int();
        ROLL_LIMIT = this->get_parameter("roll_limit").as_int();
        PITCH_LIMIT = this->get_parameter("pitch_limit").as_int();
        YAW_LIMIT = this->get_parameter("yaw_limit").as_int();
        fr_joint_dir = this->get_parameter("fr_joint_directions").as_integer_array();
        fl_joint_dir = this->get_parameter("fl_joint_directions").as_integer_array();
        br_joint_dir = this->get_parameter("br_joint_directions").as_integer_array();
        bl_joint_dir = this->get_parameter("bl_joint_directions").as_integer_array();
    
        // RCLCPP_INFO(this->get_logger(), "Loaded parameters: \n \t L1=%i \t L2=%i \t L3=%i, \t fr joint dir: %i, %i, %i", L1, L2, L3, fr_joint_dir[0], fr_joint_dir[1], fr_joint_dir[2]);
    }


    float* leg_ik(geometry_msgs::msg::Point p)
    {
        // RCLCPP_INFO(this->get_logger(), "IK_func got the point: %f, %f, %f", p.x, p.y, p.z); // for debuging
        // th1
        float* joint_angles_ = new float[3];
        float r_yz_ = sqrtf(pow(p.y,2) + pow(p.z,2));
        float sin_th1_plus_alpha_ = L1/r_yz_;
        float cos_th1_plus_alpha_ = sqrtf(1 - pow(sin_th1_plus_alpha_,2));
        joint_angles_[0] = atan2f(sin_th1_plus_alpha_, cos_th1_plus_alpha_) - atan2f(p.y, -p.z);
        // th3
        float cos_th3_ = (pow(p.x,2) + pow(p.y,2) + pow(p.z,2) - pow(L1,2) - pow(L2,2) - pow(L3,2))\
                        /(2*L2*L3);
        float sin_th3_ = sqrtf(1 - pow(cos_th3_,2));
        joint_angles_[2] = atan2f(sin_th3_, cos_th3_);
        // th2
        float a_ = L3*sinf(joint_angles_[2]);
        float b_ = L2 + L3*cosf(joint_angles_[2]);
        float c_ = sqrtf(pow(a_,2) + pow(b_,2));
        float beta_ = atan2f(b_, a_);
        float sin_th2_minus_beta_ = p.x/c_;
        float cos_th2_minus_beta_ = sqrtf(1 - pow(p.x/c_, 2));
        float th2_minus_beta_ = atan2f(sin_th2_minus_beta_, cos_th2_minus_beta_);
        joint_angles_[1] = th2_minus_beta_ + beta_;
        // RCLCPP_INFO(this->get_logger(), "IK_func calculated angles: %f, %f, %f", joint_angles_[0], joint_angles_[1], joint_angles_[2]); // for debuging
        return joint_angles_;
    }

    void calculate_FR_joint_positions(geometry_msgs::msg::Point p){
        float* jp_fr = leg_ik(p);
        for(int i=0; i<3; i++){
            if(!std::isnan(jp_fr[i])){
                joint_positions_.fr[i] = fr_joint_dir[i]*jp_fr[i];
                is_valid_fr = true;
            }
            else{
                is_valid_fr = false;
                break;
            }
        }
    }

    void calculate_FL_joint_positions(geometry_msgs::msg::Point p){
        float* jp_fl = leg_ik(p);
        for(int i=0; i<3; i++){
            if(!std::isnan(jp_fl[i])){
                joint_positions_.fl[i] = fl_joint_dir[i]*jp_fl[i];
                is_valid_fl = true;
            }
            else{
                is_valid_fl = false;
                break;
            }
        }
    }

    void calculate_BR_joint_positions(geometry_msgs::msg::Point p){
        float* jp_br = leg_ik(p);
        for(int i=0; i<3; i++){
            if(!std::isnan(jp_br[i])){
                joint_positions_.br[i] = br_joint_dir[i]*jp_br[i];
                is_valid_br = true;
            }
            else{
                is_valid_br = false;
                break;
            }
        }
    }

    void calculate_BL_joint_positions(geometry_msgs::msg::Point p){
        float* jp_bl = leg_ik(p);
        for(int i=0; i<3; i++){
            if(!std::isnan(jp_bl[i])){
                joint_positions_.bl[i] = bl_joint_dir[i]*jp_bl[i];
                is_valid_bl = true;
            }
            else{
                is_valid_bl = false;
                break;
            }
        }
    }

    void calculate_joint_position(hyperdogv2_msgs::msg::FootPositions::SharedPtr msg){
        calculate_FR_joint_positions(msg->fr);
        calculate_FL_joint_positions(msg->fl);
        calculate_BR_joint_positions(msg->br);
        calculate_BL_joint_positions(msg->bl);
        if(is_valid_fr && is_valid_fl && is_valid_br && is_valid_bl){
            previous_joint_positions_ = joint_positions_;
        }
    }
        

    void publish_joint_angles(hyperdogv2_msgs::msg::FootPositions::SharedPtr msg)
    {
        calculate_joint_position(msg);
        if(is_valid_fr && is_valid_fl && is_valid_br && is_valid_bl){
            joint_positions_publisher_->publish(joint_positions_);
        }
        else{
            if(!is_valid_fr){
                RCLCPP_INFO(this->get_logger(), "FR leg desired position (%f, %f, %f) is out of range\n", msg->fr.x, msg->fr.y, msg->fr.z);
            }
            if(!is_valid_fl){
                RCLCPP_INFO(this->get_logger(), "FL leg desired position (%f, %f, %f) is out of range\n", msg->fl.x, msg->fl.y, msg->fl.z);
            }
            if(!is_valid_br){
                RCLCPP_INFO(this->get_logger(), "BR leg desired position (%f, %f, %f) is out of range\n", msg->br.x, msg->br.y, msg->br.z);
            }
            if(!is_valid_bl){
                RCLCPP_INFO(this->get_logger(), "BL leg desired position (%f, %f, %f) is out of range\n", msg->bl.x, msg->bl.y, msg->bl.z);
            }
        }
        
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
        params_update_timer_ = this->create_wall_timer(1000ms, std::bind(&HyperdogIK::read_parameters, this));
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


