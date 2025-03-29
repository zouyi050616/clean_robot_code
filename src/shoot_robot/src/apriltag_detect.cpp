#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>

class AprilTagController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;

    // PID控制参数
    const double Kp = 0.5;                  // 比例系数
    const double target_x_tolerance = 0.05; // X轴位置容忍误差

    bool should_exit_ = false;

public:
    AprilTagController()
    {
        // 初始化订阅者和发布者
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        geometry_msgs::Twist cmd_vel;
        bool target_found = false;

        for (const auto &detection : msg->detections)
        {
            if (detection.id[0] == 5)
            {
                double current_x = detection.pose.pose.pose.position.x;
                if (fabs(current_x) < target_x_tolerance)
                {
                    // 触发退出条件
                    should_exit_ = true;
                    ros::shutdown(); // 终止ROS通信[1,7](@ref)
                    return;          // 直接退出回调函数
                }
                else if (fabs(current_x) > target_x_tolerance)
                {
                    cmd_vel.angular.z = Kp * (-current_x);
                    target_found = true;
                }
                break;
            }
        }

        if (!target_found)
            cmd_vel.angular.z = 0;
        cmd_vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_controller");
    AprilTagController controller;
    // 非阻塞式循环
    ros::Rate loop_rate(10); // 控制循环频率（10Hz）
    while (ros::ok())
    {
        ros::spinOnce(); // 处理回调队列
        loop_rate.sleep();
    }
    // 退出前的清理工作（可选）
    ROS_INFO("Node shutdown gracefully");
    return 0;
}