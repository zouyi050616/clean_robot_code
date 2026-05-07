#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <thread> 
#include <cmath>
double tag_x = 0.0;
double tag_y = 0.0;
double tag_yaw = 0.0;
int count = 0;
int re_grab_count = 0;
int grab_flag = 0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

void printRobotPose(tf2_ros::Buffer &tfBuffer)
{
    try
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("base_link", "tag_1", ros::Time(0), ros::Duration(1.0));

        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y;

        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        if (tag_x != x and tag_y != y)
        {
        	tag_x = x;
        	tag_y = y;
        	tag_yaw = atan2(y, x) * 180.0 / M_PI;
		count = 0;
        	grab_flag = 0;
        }
        else
        {
        	count++;
        	if(count > 10)
        	{
         		grab_flag = 1;
         		count = 0;
         		
         	}
        }

        ROS_INFO("Current Pose: x=%f, y=%f, yaw=%f degrees, grab=%d", x, y, tag_yaw, grab_flag);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform map to base_link: %s", ex.what());
    }
}

void printRobotPoseLoop(tf2_ros::Buffer *tfBuffer)
{
    ros::Rate rate(10); 
    while (ros::ok())
    {
        printRobotPose(*tfBuffer);
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    // TF listener setup
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    std::thread tf_thread(printRobotPoseLoop, &tfBuffer);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // Parameters
    double grab_1_x = 1.90, grab_1_y = -1.83;
    double grab_2_x = 1.90, grab_2_y = -3.05;
    double offset_left = 0.4, offset_right = 0.4;

    nh.getParam("/complete_flow_node/grab_1_x", grab_1_x);
    nh.getParam("/complete_flow_node/grab_1_y", grab_1_y);
    nh.getParam("/complete_flow_node/grab_2_x", grab_2_x);
    nh.getParam("/complete_flow_node/grab_2_y", grab_2_y);
    nh.getParam("/complete_flow_node/offset_left", offset_left);
    nh.getParam("/complete_flow_node/offset_right", offset_right);

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_close");
    upros_message::ArmPosition move_srv;

    move_base_msgs::MoveBaseGoal goal;
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;
    int count = 0;


    vel_msg.linear.x = 0.5;
    count = 0;
    while (ros::ok() && count < 8)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);




    // ---------------------- Goal 1 (Grab)
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = grab_1_x;
    goal.target_pose.pose.position.y = grab_1_y;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.stamp = ros::Time::now();
    // printRobotPose(tfBuffer);
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal 1 !!!");
    ac.waitForResult();
    
    const double search_speed =0.2;  // 统一的搜索速度
    const double return_speed =0.2; // 统一的返回速度

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal 1 Reached!");
        
        vel_msg.linear.x = 0.07;   //0.07
        count = 0;
        while (ros::ok())
        {
            if (tag_x >= 0.25)  //0.31
		{
			pub.publish(vel_msg);
			loop_rate.sleep();
		}
		else
		{
			vel_msg.linear.x = 0.0;
			pub.publish(vel_msg);
			break;
		}
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);

        while(ros::ok())
        {
		if (grab_flag == 0 and re_grab_count <= 4 and tag_x != 0.0)
		{
			ROS_INFO("retry");
			if (tag_x >= 0.25)
			{
				vel_msg.linear.x = 0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			else
			{
				vel_msg.linear.x = -0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			system("roslaunch clean_desktop_robot arm_grab.launch");
			re_grab_count++;
		}    
		else
		{
			break;
		}
	}	
        
    }
    else
    {
        ROS_WARN("Goal 1 Failed!");
    }




    // ---------------------- Backward after grab
    vel_msg.linear.x = -0.3;
    count = 0;
    while (ros::ok() && count < 14)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);





    // ---------------------- Goal 1 (Place)
    goal.target_pose.pose.position.y = grab_1_y - offset_right;
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal 1 Trash !!!");
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal 1 Trash Reached!");
        vel_msg.linear.x = 0.1;
        count = 0;
        while (ros::ok() && count < 22)  //22
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
            count++;
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        system("roslaunch clean_desktop_robot arm_put.launch");
    }

    vel_msg.linear.x = -0.3;
    count = 0;
    while (ros::ok() && count < 16)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    
    tag_x = 0.0;
    tag_y = 0.0;
    tag_yaw = 0.0;





    // ---------------------- Goal 2 (Grab)
    goal.target_pose.pose.position.x = grab_2_x;
    goal.target_pose.pose.position.y = grab_2_y;
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal 2 !!!");
    ac.waitForResult();
    
    

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
    	 ROS_INFO("Goal 2 Reached!");
	 
        vel_msg.linear.x = 0.07;
        count = 0;
        while (ros::ok())
        {
            if (tag_x >= 0.35)  //0.29
		{
			pub.publish(vel_msg);
			loop_rate.sleep();
		}
		else
		{
			vel_msg.linear.x = 0.0;
			pub.publish(vel_msg);
			break;
		}
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        system("roslaunch clean_desktop_robot arm_grab.launch");
        while(ros::ok())
        {
		if (grab_flag == 0 and re_grab_count <= 4 and tag_x != 0.0)
		{
			ROS_INFO("retry");
			if (tag_x >= 0.25)
			{
				vel_msg.linear.x = 0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			else
			{
				vel_msg.linear.x = -0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			system("roslaunch clean_desktop_robot arm_grab.launch");
			re_grab_count++;
		}    
		else
		{
			break;
		}
	}
    }





    vel_msg.linear.x = -0.3;
    count = 0;
    while (ros::ok() && count < 14)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);




    // ---------------------- Goal 2 (Place)
    goal.target_pose.pose.position.y = grab_2_y + offset_left;
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal 2 Trash !!!");
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal 2 Trash Reached!");
        vel_msg.linear.x = 0.1;
        count = 0;
        while (ros::ok() && count < 24)   //24
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
            count++;
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        system("roslaunch clean_desktop_robot arm_put.launch");
    }

    vel_msg.linear.x = -0.5;
    count = 0;
    while (ros::ok() && count < 50)    //50
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);















    // ---------------------- Return Home 1
    //goal.target_pose.pose.position.x = 2.1;
    //goal.target_pose.pose.position.y = 1.6;
    //goal.target_pose.pose.orientation.z = 0.81271;
    ///goal.target_pose.pose.orientation.w = 0.58267;
    ///goal.target_pose.header.stamp = ros::Time::now();

    //ac.sendGoal(goal);
    //ROS_INFO("Send Goal Home 1 !!!");
    //ac.waitForResult();

    //if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //{
        //ROS_INFO("Back to Home 1!");
    //}
    
    















    // ---------------------- Return Home 2
    goal.target_pose.pose.position.x = 0.2;
    goal.target_pose.pose.position.y = 0.2;
    goal.target_pose.pose.orientation.z = 0.92015;
    goal.target_pose.pose.orientation.w = -0.39157;
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ROS_INFO("Send Goal Home 2 !!!");
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back to Home 2!");
    }

    vel_msg.linear.x = 0.2;
    count = 0;
    while (ros::ok() && count < 13)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }

    return 0;
}

