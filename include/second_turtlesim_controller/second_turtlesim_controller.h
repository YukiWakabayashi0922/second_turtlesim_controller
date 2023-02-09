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

        void straight();
        void turn();
        void stop();
        double normalize_angle(double angle);

        int hz_;
        double max;
        double min;
        double init_x;
        double init_y;
        double init_theta;
        int polygon;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher pub_cmd_vel_;
        ros::Subscriber sub_pose_;

        turtlesim::Pose current_pose_;
        turtlesim::Pose old_pose_;
        geometry_msgs::Twist cmd_vel_;
};

#endif
