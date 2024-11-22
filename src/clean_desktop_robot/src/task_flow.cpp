#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <upros_core_move/CoreMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<upros_core_move::CoreMoveAction> CoreMoveClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    CoreMoveClient cc("core_move", true);

    cc.waitForServer();

    double start_x = 1.78;  // movebase目标点x
    double start_y = -0.15; // movebase目标点y
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, 1.5707);

    move_base_msgs::MoveBaseGoal goal;

    // 发送抓取导航点,先去引导点
    goal.target_pose.pose.position.x = start_x;
    goal.target_pose.pose.position.y = start_y - 0.6;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // 发送抓取导航点
    goal.target_pose.pose.position.x = start_x;
    goal.target_pose.pose.position.y = start_y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");

        // 直接开启抓取,智行W4A-T机械臂行程较长，用下面这个指令
        // system("roslaunch clean_desktop_robot arm_grab.launch");
        // 先前进一点再开启抓取，智行W4A机械臂行程较短，智行W4A用下面这个一段指令
        // 先前进一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 10)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        system("rosrun zoo_arm arm_grab_node");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // sleep(2.0);
    // upros_core_move::CoreMoveGoal goal_core;
    // goal_core.cmd = 1;
    // goal_core.target_pose.pose.position.x = start_x + 0.3;
    // goal_core.target_pose.pose.position.y = start_y;
    // cc.sendGoal(goal_core);
    // ROS_INFO("CoreMove Send Goal !!!");
    // cc.waitForResult();
    // if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("The CoreMove Goal Reached Successfully!!!");
    // }
    // else
    // {
    //     ROS_WARN("The CoreMove Goal Planning Failed for some reason");
    // }
    // goal_core.target_pose.pose.position.x = start_x + 0.6;
    // goal_core.target_pose.pose.position.y = start_y;
    // cc.sendGoal(goal_core);
    // ROS_INFO("CoreMove Send Goal !!!");
    // cc.waitForResult();
    // if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("The CoreMove Goal Reached Successfully!!!");
    //     // 开启抓取
    //     system("roslaunch clean_desktop_robot arm_grab.launch");
    //     sleep(3.0);
    //     // 开启放置
    //     system("roslaunch clean_desktop_robot arm_put.launch");
    //     sleep(3.0);
    // }
    // else
    // {
    //     ROS_WARN("The CoreMove Goal Planning Failed for some reason");
    // }

    // 发送返回导航点
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal);

    ROS_INFO("MoveBase Send Goal Home !!!");

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // 然后开始全覆盖
    // 弓字型路径的起始点
    start_x = 1.5;
    start_y = -1.0;
    // 路径生成
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_1", 10);

    double step = 0.1;
    double width = 0.6;
    double height = 0.6;

    nav_msgs::Path path;
    path.header.frame_id = "map";

    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x + start_x;
                pose.pose.position.y = y + start_y;
                path.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x + start_x;
                pose.pose.position.y = y + start_y;
                path.poses.push_back(pose);
            }
        }
    }

    path_pub.publish(path);

    // CCPP遍历起点在 map 坐标系下的坐标位置
    goal.target_pose.pose.position.x = start_x;
    goal.target_pose.pose.position.y = start_y;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ROS_INFO("Send Goal Movebase !!!");

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    for (int i = 0; i < path.poses.size(); i++)
    {
        path_pub.publish(path);
        upros_core_move::CoreMoveGoal goal;
        goal.cmd = 1;
        goal.target_pose.pose.position.x = path.poses.at(i).pose.position.x;
        goal.target_pose.pose.position.y = path.poses.at(i).pose.position.y;

        cc.sendGoal(goal);

        ROS_INFO("Sending Goal %d of %d points", i, path.poses.size());

        cc.waitForResult();

        if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The Goal 1 Reached Successfully!!!");
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
        ROS_INFO("Goal reached %d of %d points", i, path.poses.size());
    }

    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("Send Goal Home !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back !!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    return 0;
}
