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

        void straight();                       // 直進
        void turn();                           // 回転
        void stop();                           // 停止
        void run();                            // process()で実行する関数
        double normalize_angle(double angle);

        int hz_;
        int n_;  // 角の個数
        int r_;  // 外接円の半径

        int count_;
        double theta_;
        double theta_max_;
        double init_x_;
        double init_y_;
        double length_;
        double length_max_;
        double theta_base_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber pose_sub_;

        turtlesim::Pose current_pose_;
        geometry_msgs::Twist cmd_vel_;
};

#endif
