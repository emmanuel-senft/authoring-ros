#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <array>
#include <cmath>
#include <Eigen/Core>
#include <adept.h>
#include <adept_arrays.h>

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
    return trans;
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
            transformFromDH(   0.068,   0.035,  M_PI/2,    0); //Will definitely need some fine tuning
}

Eigen::Matrix<double, 4, 4> Camtransform(array<double, 6> q) {
    // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
    return  transformFromDH(       0,   0.333,       0, q[0]) *
            transformFromDH(       0,       0, -M_PI/2, q[1]) *
            transformFromDH(       0, 0.31599,  M_PI/2, q[2]) *
            transformFromDH( 0.08249,       0,  M_PI/2, q[3]) *
            transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]) *
            transformFromDH(       0,       0,  M_PI/2, q[5]) *
            transformFromDH(  0.0879,       0,  0, 0);
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

adept::aMatrix44 transformFromDHA(double a, double d, double alpha, adept::adouble theta) {
    adept::aMatrix44 trans;
    trans(0,0) = cos(theta);
    trans(1,0) = sin(theta) * cos(alpha);
    trans(2,0) = sin(theta) * sin(alpha);
    trans(3,0) = 0;

    trans(0,1) = -sin(theta);
    trans(1,1) = cos(theta) * cos(alpha);
    trans(2,1) = cos(theta) * sin(alpha);
    trans(3,1) = 0;
    
    trans(0,2) = 0;
    trans(1,2) = -sin(alpha);
    trans(2,2) = cos(alpha);
    trans(3,2) = 0;
    
    trans(0,3) = a;
    trans(1,3) = -d * sin(alpha);
    trans(2,3) = d * cos(alpha);
    trans(3,3) = 1;
    return trans;
}

adept::aMatrix44 EEtransform(array<adept::adouble, 7> q) {
    // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
    return  transformFromDHA(       0,   0.333,       0, q[0]) **
            transformFromDHA(       0,       0, -M_PI/2, q[1]) **
            transformFromDHA(       0, 0.31599,  M_PI/2, q[2]) **
            transformFromDHA( 0.08249,       0,  M_PI/2, q[3]) **
            transformFromDHA(-0.08249,   0.384, -M_PI/2, q[4]) **
            transformFromDHA(       0,       0,  M_PI/2, q[5]) **
            transformFromDHA(  0.0879,       0,  M_PI/2, q[6]) **
            transformFromDHA(       0,  0.1069,       0,    0);
}

adept::aMatrix44 Camtransform(array<adept::adouble, 6> q) {
    // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
    return  transformFromDHA(       0,   0.333,       0, q[0]) **
            transformFromDHA(       0,       0, -M_PI/2, q[1]) **
            transformFromDHA(       0, 0.31599,  M_PI/2, q[2]) **
            transformFromDHA( 0.08249,       0,  M_PI/2, q[3]) **
            transformFromDHA(-0.08249,   0.384, -M_PI/2, q[4]) **
            transformFromDHA(       0,       0,  M_PI/2, q[5]) **
            transformFromDHA(  0.0879,       0,  0, 0);
}

array<double, 42> pandaAutoJacobian(array<double, 7> q) {
    adept::Stack stack;
    array<adept::adouble, 7> q_ad;
    for (int i = 0; i < q.size(); i++) q_ad[i] = q[i];
    stack.new_recording();
    adept::aMatrix44 trans = EEtransform(q_ad);
    stack.independent(q_ad.data(), 7);

    stack.dependent(trans(0, 3)); // x_dot
    stack.dependent(trans(1, 3)); // y_dot
    stack.dependent(trans(2, 3)); // z_dot
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            stack.dependent(trans(i,j));
            R(i,j) = trans(i,j).value();
        }
    }
    Eigen::Matrix<double, 12, 7> jacobian_T;
    // This is not the jacobian WRT euler angles yet.
    // This is the jacobian WRT the transformation matrix
    stack.jacobian(jacobian_T.data());

    Eigen::Matrix<double, 6, 7> jacobian_X;
    jacobian_X.topRows(3) = jacobian_T.topRows(3);
    for (int i = 0; i < 7; i++) {
        Eigen::Matrix3d dRdq;
        for (int j = 0; j < 9; j++) {
            dRdq((int)(j / 3), j % 3) = jacobian_T(3+j,i);
        }
        auto W = dRdq * R.transpose();
        jacobian_X(3, i) = W(2,1); // w_x
        jacobian_X(4, i) = W(0,2); // w_y
        jacobian_X(5, i) = W(1,0); // w_z
    }
    array<double, 42> jacobian_array;
    Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data()) = jacobian_X;
    return jacobian_array;
}

array<double, 30> camAutoJacobian(array<double, 6> q) {
    adept::Stack stack;
    array<adept::adouble, 6> q_ad;
    for (int i = 0; i < q.size(); i++) q_ad[i] = q[i];
    stack.new_recording();
    adept::aMatrix44 trans = Camtransform(q_ad);
    stack.independent(q_ad.data(), 6);

    stack.dependent(trans(0, 3)); // x_dot
    stack.dependent(trans(1, 3)); // y_dot
    stack.dependent(trans(2, 3)); // z_dot
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            stack.dependent(trans(i,j));
            R(i,j) = trans(i,j).value();
        }
    }
    Eigen::Matrix<double, 12, 6> jacobian_T;
    // This is not the jacobian WRT euler angles yet.
    // This is the jacobian WRT the transformation matrix
    stack.jacobian(jacobian_T.data());

    Eigen::Matrix<double, 5, 6> jacobian_X;
    jacobian_X.topRows(3) = jacobian_T.topRows(3);
    for (int i = 0; i < 6; i++) {
        Eigen::Matrix3d dRdq;
        for (int j = 0; j < 9; j++) {
            dRdq((int)(j / 3), j % 3) = jacobian_T(3+j,i);
        }

        auto W = dRdq * R.transpose();
        jacobian_X(3, i) = W(2,1); // w_x
        jacobian_X(4, i) = W(0,2); // w_y
        //jacobian_X(5, i) = W(1,0); // w_z
    }
    array<double, 30> jacobian_array;
    Eigen::Map<Eigen::Matrix<double, 5, 6>>(jacobian_array.data()) = jacobian_X;
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
  cam_jacobian_array = camAutoJacobian(q);
  std::array<double, 5> velocity;
  velocity[0] = msg->linear.x;
  velocity[1] = msg->linear.y;
  velocity[2] = msg->linear.z;
  velocity[3] = msg->angular.x;
  velocity[4] = msg->angular.y;
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(velocity.data(), 5);

  Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(cam_jacobian_array.data(), 5, 6).completeOrthogonalDecomposition().solve(v);

  cout<<"Here"<<endl;
  sensor_msgs::JointState state;
  state.velocity.resize(7);
  for (size_t i = 0; i < 6; i++) {
    state.velocity[i] = jointVelocities[i];
    cout<<jointVelocities[i]<<endl;
  }
  state.velocity[6] = 0;
  angles_pub->publish(state);
}

void on_vel(const geometry_msgs::Twist::ConstPtr& msg){
  jacobian_array = pandaAutoJacobian(joints);
  
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
  ros::Subscriber cam_vel_sub = n.subscribe<geometry_msgs::Twist>("/mover/cart_vel_cam", 10, on_vel_cam);
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/viz/joint_states", 10, on_joint);

  ros::spin();
  return 0;
}
