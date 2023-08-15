#include <task2_pkg/mpc.hpp>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <tf2/utils.h>

ros::Publisher cmd_vel_pub, predict_path_pub, motion_path_pub;
nav_msgs::Path predict_path, motion_path;
geometry_msgs::PoseStamped pose_msg,pose2_msg;

Eigen::Vector3d current_state;
std::unique_ptr<Mpc> mpc_ptr(new Mpc());
Eigen::Matrix<double,1000,3> target_path;

void creatPath()
{
    //Eigen::Matrix<double,1000,3> target_path = Eigen::MatrixXd::Zero(1000, 3);
    // for (int i = 0; i < 1000; i++) {
    //     target_path(i, 0) = 0.01 * i;
    //     target_path(i, 1) = 0;
    //     target_path(i, 2) = 0;
    // }

    for (int i = 0; i < 200; i++) {
        target_path(i, 0) = 0.01 * i;
        target_path(i, 1) = 0;
        target_path(i, 2) = 0;
    }

    for (int i = 200;i<400;++i){
        target_path(i, 0) = 2;
        target_path(i, 1) = 0.01 * (i - 200);
        target_path(i, 2) = 3.1415926/2;;
    }
    for (int i = 400;i<600;++i)
    {
        target_path(i, 0) = 2-0.01*(i-400);
        target_path(i, 1) = 2;
        target_path(i, 2) = 3.1415926;
    }
    for(int i = 600;i<800;++i)
    {
        target_path(i, 0) = 0;
        target_path(i, 1) = 2-0.01*(i-600);
        target_path(i, 2) = -3.1415926/2;
    }
    for (int i = 800; i < 1000; i++) {
        target_path(i, 0) = 0.01 * (i-800);
        target_path(i, 1) = 0;
        target_path(i, 2) = 0;
    }

    motion_path.header.frame_id = "odom";
    motion_path.header.stamp = ros::Time::now();
    //cout << "got predict x" << endl;
    for (int i = 0; i < 800; i ++) {
        pose2_msg.pose.position.x = target_path(i,0);
        pose2_msg.pose.position.y = target_path(i,1);
        motion_path.poses.push_back(pose2_msg);
    }
    motion_path_pub.publish(motion_path);

}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_state << msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation);
}

void turtle_control_loop(const ros::TimerEvent &e)
{
    static int path_count = 1;
    if(path_count == 800)
    {
        //stop
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel);
        //path_count = 0;
    }
    else{
    Eigen::Matrix<double,STATESIZE,1> target_x = target_path.block(path_count, 0, 1, STATESIZE).transpose();
    Eigen::Matrix<double,STATESIZE,1> x0 = current_state - target_x;
    if(x0(2)>M_PI)
    x0(2) -= 2*M_PI;
    if(x0(2)< -M_PI)
    x0(2) += 2*M_PI;
    Eigen::Matrix3Xd x_ref = target_path.block(path_count, 0, MPCWINDOW+1, STATESIZE).transpose();
    x_ref.row(0) -= Eigen::VectorXd::Constant(x_ref.cols(), target_x(0));
    x_ref.row(1) -= Eigen::VectorXd::Constant(x_ref.cols(), target_x(1));
    x_ref.row(2) -= Eigen::VectorXd::Constant(x_ref.cols(), target_x(2)); 
    
    mpc_ptr->solveMpc(x0,x_ref,current_state);
    Eigen::Vector2d control = mpc_ptr->getControlCmd();
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = control(0);
    cmd_vel.angular.z = control(1);
    cmd_vel_pub.publish(cmd_vel);

    ROS_INFO("###########path cnt:%d##############",path_count);
    ROS_INFO("current pos:%f,%f",current_state(0),current_state(1));
    ROS_INFO("target pos:%f,%f",target_x(0),target_x(1));
    ROS_INFO("control cmd:%f,%f",control(0),control(1));
    //ROS_INFO("test:%f,%f,%f,%f",x_ref(15,0),x_ref(15,1),x_ref(16,0),x_ref(16,1));
    //std::cout<<"x_ref:"<<x_ref<<std::endl;

    path_count++;

    predict_path.header.frame_id = "odom";
    predict_path.header.stamp = ros::Time::now();
    auto predict_states = mpc_ptr->getPredictState();
    //cout << "got predict x" << endl;
    for (int i = 0; i < predict_states.size(); i += 3) {
        pose_msg.pose.position.x = predict_states[i] + target_x(0);
        pose_msg.pose.position.y = predict_states[i + 1] + target_x(1);
        predict_path.poses.push_back(pose_msg);
    }
    predict_path_pub.publish(predict_path);
    predict_path.poses.clear();

    // motion_path.header.frame_id = "odom";
    // motion_path.header.stamp = ros::Time::now();
    // //cout << "got predict x" << endl;
    // for (int i = 0; i < 800; i ++) {
    //     pose2_msg.pose.position.x = target_path(i,0);
    //     pose2_msg.pose.position.y = target_path(i,1);
    //     motion_path.poses.push_back(pose2_msg);
    // }
    // motion_path_pub.publish(motion_path);

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_tracking_node");
    ros::NodeHandle nh;

    //mpc_ptr.reset(new Mpc());

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
    motion_path_pub = nh.advertise<nav_msgs::Path>("/motion_path", 10);
    ros::Duration(1).sleep();
    creatPath();

    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &odomCallback);
    ros::Timer control_cmd_pub = nh.createTimer(ros::Duration(0.1),turtle_control_loop);

    ros::spin();
    return 0;

}
