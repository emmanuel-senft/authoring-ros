#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <panda_ros_msgs/JointPose.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <array>
#include <cmath>
#include <Eigen/Core>

using namespace std;

array<double,42> jacobian_array;
array<double,30> cam_jacobian_array;
array<double, 7> joints;
ros::Publisher* angles_pub = NULL;

Eigen::Matrix<double, 4, 4> transformFromDH(double a, double d, double alpha, double theta) {
    Eigen::MatrixXd trans(4,4);
    // Column major modified DH parameters
    trans << cos(theta), sin(theta) * cos(alpha), sin(theta) * sin(alpha), 0,
            -sin(theta), cos(theta) * cos(alpha), cos(theta) * sin(alpha), 0,
                      0,             -sin(alpha),              cos(alpha), 0,
                      a,         -d * sin(alpha),          d * cos(alpha), 1;          
    return trans.transpose();
}

Eigen::Matrix<double, 4, 4> transformDerivativeFromDH(double a, double d, double alpha, double theta) {
    Eigen::MatrixXd trans(4,4);
    // Column major modified DH parameters
    trans << -sin(theta), cos(theta) * cos(alpha), cos(theta) * sin(alpha), 0,
              -cos(theta),-sin(theta) * cos(alpha),-sin(theta) * sin(alpha), 0,
                      0,                        0,                       0, 0,
                      0,                        0,                       0, 0;
    return trans.transpose();
}

Eigen::Matrix<double, 4, 4> EEtransform(array<double, 7> q) {
    // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
    return  transformFromDH(       0,   0.333,       0, q[0]) *
            transformFromDH(       0,       0, -M_PI/2, q[1]) *
            transformFromDH(       0, 0.31599,  M_PI/2, q[2]) *
            transformFromDH( 0.08249,       0,  M_PI/2, q[3]) *
            transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]) *
            transformFromDH(       0,       0,  M_PI/2, q[5]) *
            transformFromDH(  0.0879,       0,  M_PI/2, q[6]) *
            transformFromDH(      0.,   0.275,     0,    0); //Will definitely need some fine tuning
}

Eigen::Matrix<double, 4, 4> Camtransform(array<double, 6> q) {
    // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
    return  transformFromDH(       0,   0.333,       0, q[0]) *
            transformFromDH(       0,       0, -M_PI/2, q[1]) *
            transformFromDH(       0, 0.31599,  M_PI/2, q[2]) *
            transformFromDH( 0.08249,       0,  M_PI/2, q[3]) *
            transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]) *
            transformFromDH(       0,       0,  M_PI/2, q[5]) *
            transformFromDH(   0.068,   0.035,       0,    0);
}

array<double, 16> calculatePandaEE(array<double, 7> q){
    array<double, 16> transform{};
    // Pointer magic
    Eigen::Map<Eigen::MatrixXd>(transform.data(), 4, 4) = EEtransform(q);
    return transform;
}

array<double, 16> calculateCam(array<double, 6> q){
    array<double, 16> transform{};
    // Pointer magic
    Eigen::Map<Eigen::MatrixXd>(transform.data(), 4, 4) = Camtransform(q);
    return transform;
}

array<double, 42> pandaJacobian(array<double, 7> q){
    array<Eigen::Matrix<double, 4, 4>, 8> transforms {
        transformFromDH(       0,   0.333,       0, q[0]),
        transformFromDH(       0,       0, -M_PI/2, q[1]),
        transformFromDH(       0, 0.31599,  M_PI/2, q[2]),
        transformFromDH( 0.08249,       0,  M_PI/2, q[3]),
        transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
        transformFromDH(       0,       0,  M_PI/2, q[5]),
        transformFromDH(  0.0879,       0,  M_PI/2, q[6]),
        transformFromDH(       0,   0.275,       0,    0)
    };
    array<Eigen::Matrix<double, 4, 4>, 7> transforms_derivative {
        transformDerivativeFromDH(       0,   0.333,       0, q[0]),
        transformDerivativeFromDH(       0,       0, -M_PI/2, q[1]),
        transformDerivativeFromDH(       0, 0.31599,  M_PI/2, q[2]),
        transformDerivativeFromDH( 0.08249,       0,  M_PI/2, q[3]),
        transformDerivativeFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
        transformDerivativeFromDH(       0,       0,  M_PI/2, q[5]),
        transformDerivativeFromDH(  0.0879,       0,  M_PI/2, q[6])
    };

    auto trans = EEtransform(q);
    auto R = trans.topLeftCorner(3, 3);

    Eigen::Matrix<double, 6, 7> jacobian = Eigen::MatrixXd::Zero(6, 7);
    for (int j = 0; j < 7; j++) {
        Eigen::Matrix4d jac = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < transforms.size(); i++) {
            if (i == j) {
                jac = jac * transforms_derivative[i];
            } else {
                jac = jac * transforms[i];
            }
        }
        jacobian(0, j) = jac(0, 3);
        jacobian(1, j) = jac(1, 3);
        jacobian(2, j) = jac(2, 3);

        auto W = jac.topLeftCorner(3,3) * R.transpose();
        jacobian(3, j) = W(2,1); // w_x
        jacobian(4, j) = W(0,2); // w_y
        jacobian(5, j) = W(1,0); // w_z
    }
    
    array<double, 42> jacobian_array{};
    // Pointer magic
    Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 6, 7) = jacobian;
    return jacobian_array;
}

array<double, 30> camJacobian(array<double, 6> q){
    array<Eigen::Matrix<double, 4, 4>, 7> transforms {
        transformFromDH(       0,   0.333,       0, q[0]),
        transformFromDH(       0,       0, -M_PI/2, q[1]),
        transformFromDH(       0, 0.31599,  M_PI/2, q[2]),
        transformFromDH( 0.08249,       0,  M_PI/2, q[3]),
        transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
        transformFromDH(       0,       0,  M_PI/2, q[5]),
        transformFromDH(   0.068,   0.035,       0,    0)
    };
    array<Eigen::Matrix<double, 4, 4>, 6> transforms_derivative {
        transformDerivativeFromDH(       0,   0.333,       0, q[0]),
        transformDerivativeFromDH(       0,       0, -M_PI/2, q[1]),
        transformDerivativeFromDH(       0, 0.31599,  M_PI/2, q[2]),
        transformDerivativeFromDH( 0.08249,       0,  M_PI/2, q[3]),
        transformDerivativeFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
        transformDerivativeFromDH(       0,       0,  M_PI/2, q[5])
    };

    auto trans = Camtransform(q);
    auto R = trans.topLeftCorner(3, 3);

    Eigen::Matrix<double, 5, 6> jacobian = Eigen::MatrixXd::Zero(5, 6);
    for (int j = 0; j < 6; j++) {
        Eigen::Matrix4d jac = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < transforms.size(); i++) {
            if (i == j) {
                jac = jac * transforms_derivative[i];
            } else {
                jac = jac * transforms[i];
            }
        }
        jacobian(0, j) = jac(0, 3);
        jacobian(1, j) = jac(1, 3);
        jacobian(2, j) = jac(2, 3);

        auto W = jac.topLeftCorner(3,3) * R.transpose();
        jacobian(3, j) = W(2,1); // w_x
        jacobian(4, j) = W(0,2); // w_y
        //jacobian(5, j) = W(1,0); // w_z
    }
    
    array<double, 30> jacobian_array{};
    // Pointer magic
    Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 5, 6) = jacobian;
    return jacobian_array;
}


void on_joint(const sensor_msgs::JointState::ConstPtr& msg){
  for (size_t i = 0; i < 7; i++) {
    joints[i] = msg->position[i];
  }
}

void on_vel_cam(const geometry_msgs::Twist::ConstPtr& msg){
  array<double, 6> q;
  for (size_t i = 0; i < 6; i++) {
    q[i] = joints[i];
  }
  std::array<double, 5> velocity;
  velocity[0] = msg->linear.x;
  velocity[1] = msg->linear.y;
  velocity[2] = msg->linear.z;
  velocity[3] = msg->angular.x;
  velocity[4] = msg->angular.y;
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(velocity.data(), 5);
  
  //auto start = high_resolution_clock::now();
  cam_jacobian_array = camJacobian(q); 
  Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(cam_jacobian_array.data(), 5, 6).completeOrthogonalDecomposition().solve(v);
  //auto stop = high_resolution_clock::now(); 
  //auto duration = duration_cast<microseconds>(stop - start); 
  //cout << duration.count() <<" us" << endl; 

  panda_ros_msgs::JointPose pose;
  pose.joint_pose.resize(7);
  for (size_t i = 0; i < 6; i++) {
    pose.joint_pose[i] = jointVelocities[i];
  }
  pose.joint_pose[6] = 0;
  angles_pub->publish(pose);
}

void on_vel(const geometry_msgs::Twist::ConstPtr& msg){
  //auto start = high_resolution_clock::now(); 
  jacobian_array = pandaJacobian(joints);
  
  std::array<double, 6> velocity;
  velocity[0] = msg->linear.x;
  velocity[1] = msg->linear.y;
  velocity[2] = msg->linear.z;
  velocity[3] = msg->angular.x;
  velocity[4] = msg->angular.y;
  velocity[5] = msg->angular.z;
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(velocity.data(), 6);
  Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 6, 7).completeOrthogonalDecomposition().solve(v);
  //auto stop = high_resolution_clock::now(); 
  //auto duration = duration_cast<microseconds>(stop - start); 
  //cout << duration.count() <<" us" << endl; 

  panda_ros_msgs::JointPose pose;
  pose.joint_pose.resize(7);
  for (size_t i = 0; i < 7; i++) {
    pose.joint_pose[i] = jointVelocities[i];
  }
  angles_pub->publish(pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jacob");

  ros::NodeHandle n;

  angles_pub = new ros::Publisher(n.advertise<panda_ros_msgs::JointPose>("/jacob/joint_vel", 10));

  ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("/simulator/cart_vel", 1, on_vel);
  ros::Subscriber cam_vel_sub = n.subscribe<geometry_msgs::Twist>("/simulator/cart_vel_cam", 1, on_vel_cam);
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/simulator/joint_states", 10, on_joint);

  ros::spin();
  return 0;
}
