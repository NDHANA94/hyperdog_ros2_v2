/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>
#include <tgmath.h>

#define L1 104
#define L2 150
#define L3 150

#define PI 3.14159265358979323846
#define J2_MIN 30*PI/180
#define J2_MAX 170*PI/180

using namespace std;

class Point{
    public:
        float x;
        float y;
        float z;
};

float* ik(Point p){
    bool is_any_nan = false;
    float* joint_angles = new float[3];
    float a, D, th0, th1, th2_1, th2_2;
    
    a = sqrt(pow(p.x,2) + pow(p.y,2) - pow(L1, 2));
    D = (pow(a,2) + pow(p.z,2) - pow(L2,2) - pow(L3,2)) / (2*L2*L3);
    th2_1 = atan2f(-sqrt(1-pow(D,2)), D);
    th2_2 = atan2f(sqrt(1-pow(D,2)), D);

    joint_angles[0] = atan2f(-p.y, p.x) - atan2f(a, -L1);
    joint_angles[2] = th2_2; // or th2_2 of other configuration
    joint_angles[1] = atan2f(p.z, a) - atan2f(L3*sin(joint_angles[2]), L2 + L3*cos(joint_angles[2]));

    return joint_angles;
}

int main()
{   Point p;
    p.x = 104;
    p.y = 80;
    p.z = 0;
    float* joint_angles = ik(p);
    cout<<"joint angles:  "<< joint_angles[0] << ", " << joint_angles[1]<< ", "<< joint_angles[2] << endl;
    return 0;
}
