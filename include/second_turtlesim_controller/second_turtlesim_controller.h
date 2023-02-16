#ifndef SECOND_TURTLESIM_CONTROLLER_H
#define SECOND_TURTLESIM_CONTROLLER_H

#include <ros/ros.h>
// #include <std_msgs/String.h>
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

        void straight();
        void turn();
        void stop();
        double normalize_angle(double angle);

        int hz_;
        int n_;
        int r_;

        double theta_ = 0.0;
        double theta_max_ = 0.0;
        double init_x_ = 5.54444;
        double init_y_ = 5.54444;
        double length_ = 0.0;
        double corner_ = 0.0;
        double length_max_ = 0.0;
        int count_ = 0;


        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher pub_cmd_vel_;
        ros::Subscriber sub_pose_;

        turtlesim::Pose current_pose_;
        turtlesim::Pose old_pose_;
        geometry_msgs::Twist cmd_vel_;
        // std_msgs::String msg_;
};

#endif
