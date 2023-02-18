#include "second_turtlesim_controller/second_turtlesim_controller.h"

SecondTurtlesimController::SecondTurtlesimController():private_nh_("~")
{
    private_nh_.param("hz", hz_, {100});
    private_nh_.param("n", n_, {4});
    private_nh_.param("r", r_, {2});

    count_ = 0;
    theta_ = 0.0;
    theta_max_ = 0.0;
    init_x_ = 5.54444;
    init_y_ = 5.54444;
    length_ = 0.0;
    length_max_ = 0.0;
    theta_base_ = 0.0;
    current_pose_.x = 5.54444;
    current_pose_.y = 5.54444;
    theta_base_ = 2.0*M_PI / n_;
    length_max_ = sqrt(2*r_*r_ - 2*r_*r_*cos(theta_base_));

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    pose_sub_ = nh_.subscribe("/turtle1/pose", 1, &SecondTurtlesimController::pose_callback, this);
}

void SecondTurtlesimController::pose_callback(const turtlesim::Pose::ConstPtr &msg)
{
    current_pose_ = *msg;
}

void SecondTurtlesimController::straight()
{
    cmd_vel_.linear.x = 1.5;
    cmd_vel_.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel_);
}

void SecondTurtlesimController::turn()
{
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = M_PI/6;
    cmd_vel_pub_.publish(cmd_vel_);
}

void SecondTurtlesimController::stop()
{
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel_);
}

double SecondTurtlesimController::normalize_angle(double angle)
{
    if (angle < 0.0)
    {
        angle += 2*M_PI;
    }
    return angle;
}

void SecondTurtlesimController::run()
{
    theta_ = normalize_angle(current_pose_.theta);
    theta_max_ = theta_base_ * (count_ + 1);
    length_ = hypot(current_pose_.x - init_x_, current_pose_.y - init_y_);

    if (length_ >= length_max_)
    {
        if (theta_<= theta_max_ and count_ < n_-1)
        {
            turn();
        }
        else
        {
            init_x_ = current_pose_.x;
            init_y_ = current_pose_.y;
            count_++;
        }
    }
    else
    {
        straight();
    }
}

void SecondTurtlesimController::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        run();
        if (count_ == n_)
        {
            stop();
            std::cout << "finish!!!" << std::endl;
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "second_turtlesim_controller");
    SecondTurtlesimController kame;
    kame.process();
    return 0;
}
