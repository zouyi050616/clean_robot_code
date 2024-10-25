#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <upros_core_move/CoreMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

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

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    CoreMoveClient cc("core_move", true);

    cc.waitForServer();

    double start_x = 1.0;  // movebase目标点x
    double start_y = -0.3; // movebase目标点y

    move_base_msgs::MoveBaseGoal goal;

    // 发送抓取导航点
    goal.target_pose.pose.position.x = start_x;
    goal.target_pose.pose.position.y = start_y;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
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

    sleep(2.0);

    upros_core_move::CoreMoveGoal goal_core;
    goal_core.cmd = 1;
    goal_core.target_pose.pose.position.x = start_x + 0.3;
    goal_core.target_pose.pose.position.y = start_y;

    cc.sendGoal(goal_core);

    ROS_INFO("CoreMove Send Goal !!!");

    cc.waitForResult();

    if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The CoreMove Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The CoreMove Goal Planning Failed for some reason");
    }

    goal_core.target_pose.pose.position.x = start_x + 0.6;
    goal_core.target_pose.pose.position.y = start_y;

    cc.sendGoal(goal_core);

    ROS_INFO("CoreMove Send Goal !!!");

    cc.waitForResult();

    if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The CoreMove Goal Reached Successfully!!!");

        // 开启抓取
        system("roslaunch clean_desktop_robot arm_grab.launch");

        sleep(3.0);

        // 开启放置
        system("roslaunch clean_desktop_robot arm_put.launch");

        sleep(3.0);
    }
    else
    {
        ROS_WARN("The CoreMove Goal Planning Failed for some reason");
    }

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

    return 0;
}
