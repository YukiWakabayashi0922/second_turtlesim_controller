#ifndef SECOND_TURTLESIM_CONTROLLER_H
#define SECOND_TURTLESIM_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

class SecondTurtlesimController
{
    public:
        SecondTurtlesimController();
        void process();

    private:
        void pose_callback(const turtlesim::Pose::ConstPtr &msg);

        void straight();                       //直進
        void turn();                           //回転
        void stop();                           //停止
        double normalize_angle(double angle);

        // launchファイルで指定するパラメータ
        int hz_;
        int n_;  //角の個数
        int r_;  //外接円の半径

        int count_ = 0;
        double theta_ = 0.0;
        double theta_max_ = 0.0;
        double init_x_ = 5.54444;
        double init_y_ = 5.54444;
        double length_ = 0.0;
        double length_max_ = 0.0;
        double corner_ = 0.0;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber pose_sub_;

        turtlesim::Pose current_pose_;
        geometry_msgs::Twist cmd_vel_;
};

#endif
