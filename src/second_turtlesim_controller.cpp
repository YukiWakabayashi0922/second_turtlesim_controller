#include "second_turtlesim_controller/second_turtlesim_controller.h"

SecondTurtlesimController::SecondTurtlesimController():private_nh_("~")
{
    private_nh_.param("hz", hz_, {100});
    private_nh_.param("max", max, {0.0});
    private_nh_.param("min", min, {0.0});
    // private_nh_.param("init_x", init_x, {0.0});
    // private_nh_.param("init_y", init_y, {0.0});
    // private_nh_.param("init_theta", init_theta, {0.0});
    private_nh_.param("polygon", polygon, {0});

    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    sub_pose_ = nh_.subscribe("/turtle1/pose", 10, &SecondTurtlesimController::pose_callback, this);
}

void SecondTurtlesimController::pose_callback(const turtlesim::Pose::ConstPtr &msg)
{
    current_pose_ = *msg;
}

void SecondTurtlesimController::straight()
{
    cmd_vel_.linear.x = 1.5;
    cmd_vel_.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
    // std::cout<<"straight"<<std::endl;
}

void SecondTurtlesimController::turn()
{
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = M_PI/2;
    pub_cmd_vel_.publish(cmd_vel_);
    // std::cout<<"turn"<<std::endl;
}

void SecondTurtlesimController::stop()
{
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
}

double SecondTurtlesimController::normalize_angle(double angle)
{
    if (angle < 0.0) {
        angle += 2*M_PI;
    }
    return angle;
}

void SecondTurtlesimController::process()
{
    current_pose_.x = 5.54444;
    current_pose_.y = 5.54444;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double init_x = 5.54444;
    double init_y = 5.54444;
    // double init_theta = 0.0;
    double corner = M_PI - ((polygon - 2.0)*M_PI) / polygon;
    double length = 0.0;
    double length_max = 9 / polygon;
    int count = 0;
    double theta_max = 0.0;

    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        x = current_pose_.x;
        y = current_pose_.y;
        theta = normalize_angle(current_pose_.theta);
        // theta_max = normalize_angle(corner * (count+1));
        theta_max = corner * (count+1);

        length = hypot(x - init_x, y - init_y);
        // theta = normalize_angle(theta);

        if (length >= length_max) {
            if (theta <= theta_max and count < polygon-1) {
                turn();
            } else {
                init_x = x;
                init_y = y;
                init_theta = theta;

                // straight();

                count++;
                // std::cout<<"init_theta"<<init_theta<<std::endl;
                // std::cout<<"count: "<<count<<std::endl;
            }
        } else {
            straight();

        }

        if (count == polygon) {
            stop();
            std::cout<<"finish!!!"<<std::endl;
            break;
        }

        // std::cout<<"x: "<<x<<std::endl;
        // std::cout<<"y: "<<y<<std::endl;
        // std::cout<<"theta: "<<theta<<std::endl;
        // pub_cmd_vel_.publish(cmd_vel_);

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
