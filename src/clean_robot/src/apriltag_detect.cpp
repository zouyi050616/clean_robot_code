#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <apriltag_ros/AprilTagDetectionArray.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int tag = 0;

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    for (const auto &detection : msg->detections)
    {
        if (detection.id.size() == 1 && detection.id[0] == 1)
        {
            // detect id 1
            tag = 1;
        }
        else if (detection.id.size() == 1 && detection.id[0] == 2)
        {
            // detect id 2
            tag = 2;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 10, tagDetectionsCallback);

    move_base_msgs::MoveBaseGoal goal1; // 查询apriltag的点
    goal1.target_pose.pose.position.x = 1.0;
    goal1.target_pose.pose.position.y = 0.0;
    goal1.target_pose.pose.orientation.z = 0.0;
    goal1.target_pose.pose.orientation.w = 1.0;
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal1);
    ROS_INFO("Send Goal  1 !!!");

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 1 Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    ros::Rate rate(10);
    while (ros::ok() && tag == 0)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Tag Found!!!!");

    if (tag == 1)
    {
        move_base_msgs::MoveBaseGoal goal2; // id为1的tag要去的点
        goal2.target_pose.pose.position.x = 2.0;
        goal2.target_pose.pose.position.y = 0.3;
        goal2.target_pose.pose.orientation.z = 0.0;
        goal2.target_pose.pose.orientation.w = 1.0;
        goal2.target_pose.header.frame_id = "map";
        goal2.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal2);
        ROS_INFO("Send Goal  2 !!!");
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The Goal 2 Reached Successfully!!!");
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
    }

    if (tag == 2)
    {
        move_base_msgs::MoveBaseGoal goal3; // id为2的tag要去的点
        goal3.target_pose.pose.position.x = 0.3;
        goal3.target_pose.pose.position.y = 2.0;
        goal3.target_pose.pose.orientation.z = 0.0;
        goal3.target_pose.pose.orientation.w = 1.0;
        goal3.target_pose.header.frame_id = "map";
        goal3.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal3);
        ROS_INFO("Send Goal  3 !!!");
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The Goal 3 Reached Successfully!!!");
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
    }

    move_base_msgs::MoveBaseGoal goal4; //home
    goal4.target_pose.pose.position.x = 0.0;
    goal4.target_pose.pose.position.y = 0.0;
    goal4.target_pose.pose.orientation.z = 0.0;
    goal4.target_pose.pose.orientation.w = 1.0;
    goal4.target_pose.header.frame_id = "map";
    goal4.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal4);
    ROS_INFO("Send Goal  Home !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 3 Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    return 0;
}
