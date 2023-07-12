/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>
#include <tgmath.h>
// #include "rclcpp/rclcpp.hpp"

#define L1 104
#define L2 150
#define L3 150

#define PI 3.14159265358979323846
#define J2_MIN 30*PI/180
#define J2_MAX 170*PI/180

using namespace std;

double* joint_angs = new double[3];
bool is_valid = false;

class Point{
    public:
        float x;
        float y;
        float z;
};

float* ik(Point p){
    float* joint_angles_ = new float[3];
    float r_yz_ = sqrtf(powf(p.y,2) + powf(p.z,2));
    float sin_th1_plus_alpha_ = L1/r_yz_;
    float cos_th1_plus_alpha_ = sqrtf(1 - powf(sin_th1_plus_alpha_,2));
    joint_angles_[0] = atan2f(sin_th1_plus_alpha_, cos_th1_plus_alpha_) - atan2f(p.y, -p.z);
    // ---
    float cos_th3_ = (powf(p.x,2) + powf(p.y,2) + powf(p.z,2) - powf(L1,2) - powf(L2,2) - powf(L3,2))\
                    /(2*L2*L3);
    float sin_th3_ = sqrtf(1 - powf(cos_th3_,2));
    joint_angles_[2] = atan2f(sin_th3_, cos_th3_);
    // ---
    float a_ = L3*sinf(joint_angles_[2]);
    float b_ = L2 + L3*cosf(joint_angles_[2]);
    float c_ = sqrtf(powf(a_,2) + powf(b_,2));
    float beta_ = atan2f(b_, a_);
    float sin_th2_minus_beta_ = p.x/c_;
    float cos_th2_minus_beta_ = sqrtf(1 - powf(p.x/c_, 2));
    float th2_minus_beta_ = atan2f(sin_th2_minus_beta_, cos_th2_minus_beta_);
    joint_angles_[1] = th2_minus_beta_ + beta_;
    return joint_angles_;
}

bool update_joint_angs(Point p){
    float* j_angs = ik(p);
    for(int i = 0; i<3; i++){
        cout<<j_angs[i]<<endl;
        if(!isnan(j_angs[i])){
            joint_angs[i] = j_angs[i];
            is_valid = true;
        }
        else{
            is_valid = false;
            return 0;
            break;
        }
    }
    return 1;
}

int main()
{   Point p;
    p.x = 10;
    p.y = 104;
    p.z = -200;
    is_valid = update_joint_angs(p);
    cout<<"is valid: "<<is_valid<<endl;
    for(int i=0; i<3; i++){
        cout<<joint_angs[i] << ", ";
    }
    cout<<endl;

    // ------------------------------
    cout<<"---------------------------\n";
    // float32 x = 1.09234;

    // printf("%d", {1.0, 2.0, 3.834});
    return 0;
}
