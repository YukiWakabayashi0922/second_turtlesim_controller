#include "second_turtlesim_controller/second_turtlesim_controller.h"

SecondTurtlesimController::SecondTurtlesimController():private_nh_("~")
{
    private_nh_.param("hz", hz_, {0});
    private_nh_.param("n", n_, {0});
    private_nh_.param("r", r_, {0});

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pose_sub_ = nh_.subscribe("/turtle1/pose", 10, &SecondTurtlesimController::pose_callback, this);
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
    cmd_vel_.angular.z = M_PI/2;
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

void SecondTurtlesimController::process()
{
    current_pose_.x = 5.54444;
    current_pose_.y = 5.54444;
    corner_ = 2.0*M_PI / n_;
    length_max_ = sqrt(2*r_*r_ - 2*r_*r_*cos(corner_));

    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        theta_ = normalize_angle(current_pose_.theta);
        theta_max_ = corner_ * (count_ + 1);
        length_ = hypot(current_pose_.x - init_x_, current_pose_.y - init_y_);

        if (length_ >= length_max_)
        {
            if (theta_ <= theta_max_ and count_ < n_-1)
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

        if (count_ == n_)
        {
            stop();
            std::cout<<"finish!!!"<<std::endl;
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
