#include "clean_desktop_robot/upros_arm_driver.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

bool got_finish = false;

double start_y = 0.0;
double target_y = -0.4;

double odom_distance = 0;

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    double current_y = odom_msg->pose.pose.position.y;
    if (start_y == 0)
    {
        start_y = current_y;
    }
    if (current_y - start_y <= target_y)
    {
        got_finish = true;
        ROS_INFO("Finish!!!!!!!!");
    }
}

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_put");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 订阅Odom数据
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, odom_callback);

    // 运动控制
    ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist cmd_vel;

    // 先平移0.6米
    while (ros::ok() && !got_finish)
    {
        cmd_vel.linear.y = -0.15;
        cmd_pub_.publish(cmd_vel);
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    for (int i=0; i<20; i++)
    {
        cmd_vel.linear.y = 0.0;
        cmd_pub_.publish(cmd_vel);
        ros::spinOnce();
        ros::Duration(0.05).sleep();        
    }

    // 平移完成，开始抓取
    UPROS_ARM arm;

    sleep(3.0);

    int x = 0;
    int y = 200;
    int z = 45;
    if (arm.inverseFind(x, y, z))
    {
        ROS_INFO("Find Soulition!!!!");
    }
    else
    {
        ROS_ERROR("Cant Find Soulition!!!!");
    }

    sleep(2.0);

    arm.inverseMoveToPut(); // 逆运算规划与控制

    sleep(2.0);

    arm.inverseMoveToPut(); // 逆运算规划与控制
    
    sleep(3.0);

    arm.claw_open(); // 打开夹爪

    sleep(3.0);

    arm.go_home();

    sleep(3.0);

    arm.go_home();

    sleep(3.0);

    ros::shutdown();

    return 0;
}
