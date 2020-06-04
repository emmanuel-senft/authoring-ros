#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;
array<double,42> jacobian_array;
ros::Publisher* angles_pub = NULL;

void calculatePandaJacobian(array<double, 7> q) {
    // Calculated using Modified DH-Parameters analytically
    // using joint anlges, q:

    // Store the results in a column-wise matrix that is put in the
    // shared memory object

    // Column 0
    //(0,0)
    jacobian_array[0]=0.384*std::cos(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.31599*std::sin(q[0])*
    std::sin(q[1]) - 0.384*std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 0.08249*std::cos(q[0])*
    std::sin(q[2]) + 0.08249*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[1])*
    std::cos(q[2])*std::sin(q[0]) + 0.08249*std::cos(q[0])*std::cos(q[3])*std::sin(q[2]) + 0.08249*
    std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0]) - 0.0879*std::cos(q[0])*std::cos(q[2])*
    std::cos(q[5])*std::sin(q[4]) + 0.384*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]) +
      0.1069*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.1069*std::cos(q[0])*
      std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*
      std::sin(q[5]) - 0.0879*std::cos(q[3])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) +
      0.0879*std::cos(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
      std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.1069*std::cos(q[1])*std::cos(q[2])*
      std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*
      std::sin(q[2])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*
      std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) -
      0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.1069*
      std::cos(q[1])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*std::cos(q[4])*
      std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*
      std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0]) - 0.1069*std::cos(q[1])*std::cos(q[2])*
      std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

    //(1,0)
    jacobian_array[1]=0.31599*std::cos(q[0])*std::sin(q[1]) - 0.08249*std::sin(q[0])*std::sin(q[2]) - 0.08249*
    std::cos(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.08249*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) + 0.384*
    std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2]) + 0.384*
    std::cos(q[0])*std::cos(q[3])*std::sin(q[1]) - 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[3]) - 0.384*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*
    std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.0879*std::cos(q[0])*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) -
    0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) + 0.0879*
    std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[5])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3])*std::sin(q[5]) -
    0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.0879*std::cos(q[0])*
    std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.0879*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*
    std::sin(q[0])*std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) +
    0.1069*std::cos(q[0])*std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.1069*std::cos(q[3])*
    std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
    std::cos(q[4])*std::sin(q[5]);

    //(2,0)
    jacobian_array[2]=0.0;

    //(3,0)
    jacobian_array[3]=0.0;

    //(4,0)
    jacobian_array[4]=0.0;

    //(5,0)
    jacobian_array[5]=1.0;

    // Column 1
    //(0,1)
    jacobian_array[6]=0.31599*std::cos(q[0])*std::cos(q[1]) + 0.384*std::cos(q[0])*std::cos(q[1])*std::cos(q[3]) -
    0.08249*std::cos(q[0])*std::cos(q[2])*std::sin(q[1]) - 0.08249*std::cos(q[0])*std::cos(q[1])*std::sin(q[3]) -
    0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 0.08249*std::cos(q[0])*std::cos(q[2])*
    std::cos(q[3])*std::sin(q[1]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*std::sin(q[5]) + 0.384*
    std::cos(q[0])*std::cos(q[2])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*
    std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) +
    0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*
    std::cos(q[2])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[1])*
    std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[0])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
    0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) - 0.1069*
    std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]);

    //(1,1)
    jacobian_array[7]=0.31599*std::cos(q[1])*std::sin(q[0]) - 0.08249*std::cos(q[2])*std::sin(q[0])*std::sin(q[1]) -
    0.08249*std::cos(q[1])*std::sin(q[0])*std::sin(q[3]) + 0.384*std::cos(q[1])*std::cos(q[3])*std::sin(q[0]) -
    0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[5])*std::sin(q[0]) + 0.08249*std::cos(q[2])*std::cos(q[3])*
    std::sin(q[0])*std::sin(q[1]) + 0.0879*std::cos(q[1])*std::cos(q[3])*std::sin(q[0])*std::sin(q[5]) + 0.384*
    std::cos(q[2])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*
    std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) +
    0.1069*std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[2])*
    std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*
    std::sin(q[2])*std::sin(q[4]) + 0.1069*std::sin(q[0])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
    0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.1069*
    std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]);

    //(2,1)
    jacobian_array[8]=0.08249*std::sin(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[1])*std::cos(q[2]) - 0.384*std::cos(q[3])*
    std::sin(q[1]) - 0.31599*std::sin(q[1]) - 0.0879*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) + 0.08249*std::cos(q[1])*
    std::cos(q[2])*std::cos(q[3]) + 0.384*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[3])*std::cos(q[5])*
    std::sin(q[1]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[2])*
    std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) - 0.0879*std::cos(q[4])*
    std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) + 0.1069*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*
    std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
    std::cos(q[4])*std::cos(q[5]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

    //(3,1)
    jacobian_array[9] = -1.0*std::sin(q[0]);

    //(4,1)
    jacobian_array[10]=std::cos(q[0]);

    //(5,1)
    jacobian_array[11]=0.0;

    // Column 2
    
    //(0,2)
    jacobian_array[12]=0.384*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]) - 0.08249*std::cos(q[2])*std::sin(q[0]) - 0.08249*std::cos(q[0])*
    std::cos(q[1])*std::sin(q[2]) + 0.08249*std::cos(q[2])*std::cos(q[3])*std::sin(q[0]) + 0.08249*std::cos(q[0])*std::cos(q[1])*
    std::cos(q[3])*std::sin(q[2]) + 0.384*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[2])*
    std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) + 0.0879*
    std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) + 0.1069*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
    0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[4]) - 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*
    std::cos(q[5])*std::sin(q[0]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*
    std::cos(q[1])*std::cos(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*
    std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
    std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*
    std::cos(q[4])*std::sin(q[2])*std::sin(q[5]);

    //(1,2)
    jacobian_array[13]=0.08249*std::cos(q[0])*std::cos(q[2]) - 0.08249*std::cos(q[1])*std::sin(q[0])*std::sin(q[2]) - 0.08249*std::cos(q[0])*
    std::cos(q[2])*std::cos(q[3]) - 0.384*std::cos(q[0])*std::cos(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*
    std::sin(q[3]) + 0.08249*std::cos(q[1])*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) - 0.0879*std::cos(q[0])*std::cos(q[2])*std::sin(q[3])*
    std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.384*std::cos(q[1])*std::sin(q[0])*std::sin(q[2])*
    std::sin(q[3]) - 0.1069*std::cos(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*
    std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[1])*
    std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*
    std::sin(q[3]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::sin(q[0])*
    std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[2]) - 0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]);

    //(2,2)
    jacobian_array[14]=0.08249*std::sin(q[1])*std::sin(q[2]) - 0.08249*std::cos(q[3])*std::sin(q[1])*std::sin(q[2]) - 0.384*std::sin(q[1])*
    std::sin(q[2])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[4]) + 0.1069*std::cos(q[5])*std::sin(q[1])*
    std::sin(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[2])*std::sin(q[1])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::sin(q[1])*std::sin(q[2])*
    std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[2]) + 0.1069*std::cos(q[3])*
    std::cos(q[4])*std::sin(q[1])*std::sin(q[2])*std::sin(q[5]);

    //(3,2)
    jacobian_array[15]=std::cos(q[0])*std::sin(q[1]);

    //(4,2)
    jacobian_array[16]=std::sin(q[0])*std::sin(q[1]);

    //(5,2)
    jacobian_array[17]=std::cos(q[1]);


    // Column 3

    //(0,3)
    jacobian_array[18]=0.384*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) - 0.384*std::cos(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.08249*
    std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.08249*std::cos(q[0])*std::cos(q[3])*std::sin(q[1]) - 0.384*std::cos(q[0])*
    std::cos(q[1])*std::cos(q[2])*std::cos(q[3]) + 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) + 0.1069*
    std::cos(q[0])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.1069*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2]) -
    0.0879*std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) +
    0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) + 0.1069*std::cos(q[0])*
    std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]) + 0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*
    std::sin(q[3]) + 0.1069*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*
    std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*
    std::sin(q[3])*std::sin(q[5]);

    //(1,3)
    jacobian_array[19]=0.08249*std::cos(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.08249*std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 0.384*
    std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.384*std::cos(q[0])*std::cos(q[3])*std::sin(q[2]) - 0.384*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[3])*std::sin(q[0]) + 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[5])*std::sin(q[2]) + 0.08249*std::cos(q[1])*std::cos(q[2])*
    std::sin(q[0])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[5]) + 0.1069*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[1])*std::sin(q[3]) - 0.0879*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[3])*std::cos(q[5])*std::sin(q[0]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0])*std::sin(q[5]) + 0.0879*
    std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.0879*std::cos(q[0])*std::cos(q[4])*std::cos(q[5])*
    std::sin(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[0])*
    std::cos(q[4])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*
    std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]);

    //(2,3)
    jacobian_array[20]=0.384*std::cos(q[2])*std::cos(q[3])*std::sin(q[1]) - 0.384*std::cos(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[2])*
    std::sin(q[1])*std::sin(q[3]) - 0.0879*std::cos(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.08249*std::cos(q[1])*std::cos(q[3]) + 0.1069*
    std::cos(q[1])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) - 0.1069*std::cos(q[2])*
    std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[2])*
    std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) + 0.0879*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) + 0.1069*
    std::cos(q[2])*std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]);

    //(3,3)
    jacobian_array[21]=std::cos(q[2])*std::sin(q[0]) + std::cos(q[0])*std::cos(q[1])*std::sin(q[2]);

    //(4,3)
    jacobian_array[22]=std::cos(q[1])*std::sin(q[0])*std::sin(q[2]) - 1.0*std::cos(q[0])*std::cos(q[2]);

    //(5,3)
    jacobian_array[23]=-1.0*std::sin(q[1])*std::sin(q[2]);

    // Column 4

    //(0,4)
    jacobian_array[24]=0.0879*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) - 0.1069*std::cos(q[2])*
    std::cos(q[4])*std::sin(q[0])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*
    std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
    std::cos(q[5])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4]) - 0.0879*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*
    std::sin(q[0]) - 0.1069*std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4])*std::sin(q[5]) + 0.1069*
    std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[3])*std::cos(q[5])*std::sin(q[4]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[4])*std::sin(q[5]);

    //(1,4)
    jacobian_array[25]=0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[4])*
    std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2]) - 0.0879*std::cos(q[0])*std::cos(q[3])*
    std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) - 0.1069*
    std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*
    std::sin(q[3])*std::sin(q[4]) - 0.1069*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[1])*
    std::cos(q[2])*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
    std::sin(q[0])*std::sin(q[4])*std::sin(q[5]);

    //(2,4)
    jacobian_array[26]=0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[2]) - 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[3])*
    std::sin(q[4]) + 0.1069*std::cos(q[4])*std::sin(q[1])*std::sin(q[2])*std::sin(q[5]) - 0.1069*std::cos(q[1])*std::sin(q[3])*std::sin(q[4])*
    std::sin(q[5]) + 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[5])*std::sin(q[1])*std::sin(q[4]) + 0.1069*std::cos(q[2])*std::cos(q[3])*
    std::sin(q[1])*std::sin(q[4])*std::sin(q[5]);

    //(3,4)
    jacobian_array[27]=std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + std::cos(q[0])*std::cos(q[3])*
    std::sin(q[1]) - 1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]);

    //(4,4)
    jacobian_array[28]=std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 1.0*std::cos(q[0])*
    std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]);

    //(5,4)
    jacobian_array[29]=std::cos(q[1])*std::cos(q[3]) + std::cos(q[2])*std::sin(q[1])*std::sin(q[3]);

    // Column 5

    //(0,5)
    jacobian_array[30]=0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.1069*
    std::cos(q[0])*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[4]) + 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::sin(q[0])*
    std::sin(q[4])*std::sin(q[5]) + 0.1069*std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
    std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3])*
    std::sin(q[5]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[0])*
    std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.1069*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*
    std::sin(q[0])*std::sin(q[2]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
    std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*
    std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) - 0.0879*
    std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

    //(1,5)
    jacobian_array[31]=0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*std::sin(q[4]) + 0.0879*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[1]) - 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*
    std::sin(q[5]) + 0.1069*std::cos(q[3])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[0])*std::sin(q[2])*std::sin(q[3])*
    std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.0879*std::cos(q[1])*std::cos(q[2])*
    std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 0.1069*
    std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) - 0.1069*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[1])*
    std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
    std::sin(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0]) - 0.0879*std::cos(q[1])*
    std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

    //(2,5)
    jacobian_array[32]=0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[3])*std::sin(q[5]) + 0.1069*
    std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.0879*
    std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[2])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.1069*
    std::cos(q[5])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4]) - 0.0879*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*
    std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) + 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*
    std::sin(q[1])*std::sin(q[5]);

    //(3,5)
    jacobian_array[33]=std::cos(q[2])*std::cos(q[4])*std::sin(q[0]) + std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[2]) + 1.0*
    std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4]) - 1.0*std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) + 1.0*
    std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[4]);

    //(4,5)
    jacobian_array[34]=std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2]) - 1.0*std::cos(q[0])*std::cos(q[2])*
    std::cos(q[4]) + std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[4]) + std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
    std::sin(q[4]) + std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0])*std::sin(q[4]);

    //(5,5)
    jacobian_array[35]=std::cos(q[1])*std::sin(q[3])*std::sin(q[4]) - 1.0*std::cos(q[4])*
    std::sin(q[1])*std::sin(q[2]) - 1.0*std::cos(q[2])*std::cos(q[3])*std::sin(q[1])*std::sin(q[4]);

    // Column 6

    //(0,6)
    jacobian_array[36]=0;

    //(1,6)
    jacobian_array[37]=0;

    //(2,6)
    jacobian_array[38]=0;

    //(3,6)
    jacobian_array[39]=1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) - 1.0*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) - 1.0*std::cos(q[0])*std::cos(q[3])*
    std::cos(q[5])*std::sin(q[1]) - 1.0*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + 1.0*std::cos(q[0])*
    std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 1.0*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*
    std::sin(q[5]) + 1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

    //(4,6)
    jacobian_array[40]=1.0*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*
    std::sin(q[1]) + 1.0*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*std::sin(q[5]) + 1.0*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*
    std::sin(q[0])*std::sin(q[3]) + std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 1.0*std::cos(q[1])*
    std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
    std::sin(q[5]) + std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

    //(5,6)
    jacobian_array[41]=std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) - 1.0*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*
    std::sin(q[3]) - 1.0*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 1.0*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*
    std::sin(q[5]) - 1.0*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]);
}

void on_joint(const sensor_msgs::JointState::ConstPtr& msg){
  array<double, 7> q;
  for (size_t i = 0; i < 7; i++) {
    q[i] = msg->position[i];
  }
  calculatePandaJacobian(q);
}

void on_vel(const geometry_msgs::Twist::ConstPtr& msg){
  std::array<double, 6> velocity;
  velocity[0] = msg->linear.x;
  velocity[1] = msg->linear.y;
  velocity[2] = msg->linear.z;
  velocity[3] = msg->angular.x;
  velocity[4] = msg->angular.y;
  velocity[5] = msg->angular.z;
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(velocity.data(), 6);

  Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 6, 7).completeOrthogonalDecomposition().solve(v);

  sensor_msgs::JointState state;
  state.velocity.resize(7);
  for (size_t i = 0; i < 7; i++) {
    state.velocity[i] = jointVelocities[i];
  }
  angles_pub->publish(state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jacob");

  ros::NodeHandle n;

  angles_pub = new ros::Publisher(n.advertise<sensor_msgs::JointState>("/jacob/joint_vel", 10));

  ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("/mover/cart_vel", 10, on_vel);
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/viz/joint_states", 10, on_joint);

  ros::spin();
  return 0;
}
