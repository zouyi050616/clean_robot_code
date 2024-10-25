#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <upros_core_move/CoreMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <nav_msgs/Path.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef actionlib::SimpleActionClient<upros_core_move::CoreMoveAction> CoreMoveClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    CoreMoveClient cc("core_move", true);

    cc.waitForServer();
    
    //弓字型路径的起始点
    double start_x = 2.0;
    double start_y = 0.3;

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

    move_base_msgs::MoveBaseGoal goal1;
    move_base_msgs::MoveBaseGoal goal2;
    move_base_msgs::MoveBaseGoal goal3;

    // 待发送的 目标点 在 map 坐标系下的坐标位置
    goal1.target_pose.pose.position.x = start_x;
    goal1.target_pose.pose.position.y = start_y;
    goal1.target_pose.pose.orientation.z = 0.0;
    goal1.target_pose.pose.orientation.w = 1.0;
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal1);
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

    return 0;
}
