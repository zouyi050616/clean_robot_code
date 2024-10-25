#include "clean_desktop_robot/upros_arm_driver.h"

#include <string>
#include <stdlib.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_ik_demo");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    UPROS_ARM arm;

    sleep(3.0);

    arm.claw_open(); //先打开夹爪

    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ROS_INFO("tf coordinate transformaing....");

    // 获取tag到机械臂基坐标的坐标变换
    geometry_msgs::TransformStamped tfs_1 = buffer.lookupTransform("arm_base_link", "tag_3", ros::Time(0), ros::Duration(100.0));


    // 单位转换
    int x = -int(tfs_1.transform.translation.y * 1000);
    int y = int(tfs_1.transform.translation.x * 1000);
    int z = int(tfs_1.transform.translation.z * 1000);

    ROS_INFO("Find Tag x: %d, y: %d, z: %d ;", x, y, z);

    // 逆运算
    if (arm.inverseFind(x, y, z))
    {
        ROS_INFO("Find Soulition!!!!");
    }

    sleep(3.0);

    // 逆运动学规划
    arm.inverseMoveToGrab();

    sleep(1.0);
    arm.inverseMoveToGrab();    

    sleep(3.0);

    //关闭夹爪
    arm.claw_close(); 

    sleep(3.0);
    
    //回到零位
    arm.go_home();

    sleep(3.0);

    //回到零位
    arm.go_home();

    ros::shutdown();

    return 0;
}
