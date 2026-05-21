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
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include "upros_message/ArmPosition.h"
#include <upros_message/SingleServo.h>
#include "std_srvs/Empty.h"
#include <thread> 
#include <atomic>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <string>

// 全局变量和TF反馈

double tag_x = 0.0;
double tag_y = 0.0;
double tag_yaw = 0.0;
int count = 0;
int re_grab_count = 0;
int grab_flag = 0;
bool g_last_grab_done = false;
bool g_tag_fresh = false;

ros::Publisher *g_cmd_vel_pub = nullptr;
ros::Publisher *g_single_servo_pub = nullptr;
tf2_ros::Buffer *g_tf_buffer = nullptr;
ros::ServiceClient *g_arm_release_client = nullptr;
ros::ServiceClient *g_arm_zero_client = nullptr;
double g_quick_dump_linear_vel = 0.08;
double g_quick_dump_angular_vel = 0.25;
double g_quick_dump_rotate_timeout_margin_sec = 3.0;
double g_quick_dump_min_angular_vel = 0.35;
double g_quick_dump_fine_min_angular_vel = 0.28;
double g_quick_dump_slowdown_angle_deg = 18.0;
double g_quick_dump_rotate_tolerance_deg = 0.5;
double g_quick_dump_rotate_overshoot_tolerance_deg = 2.0;
double g_quick_dump_fine_tune_timeout_sec = 12.0;
double g_quick_dump_fine_pulse_sec = 0.12;
double g_quick_dump_fine_settle_sec = 0.25;
int g_quick_dump_release_repeat = 2;
double g_quick_dump_release_pause_sec = 0.25;
double g_tag_max_age = 0.8;
bool g_quick_dump_arm_yaw_compensation = false;
ros::Time g_last_tag_stamp;
std::atomic<bool> g_tf_thread_running(true);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

//printRobotPose和循环监听函数

void printRobotPose(tf2_ros::Buffer &tfBuffer)
{
    try
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("base_link", "tag_1", ros::Time(0), ros::Duration(1.0));

        const ros::Time tag_stamp = transformStamped.header.stamp;
        if (!tag_stamp.isZero() && (ros::Time::now() - tag_stamp).toSec() > g_tag_max_age)
        {
            g_tag_fresh = false;
            ROS_WARN_THROTTLE(1.0, "tag_1 TF is stale: age=%.3f s", (ros::Time::now() - tag_stamp).toSec());
            return;
        }
        g_tag_fresh = true;
        g_last_tag_stamp = tag_stamp;

        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y;

        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // 稳定性判断

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
        g_tag_fresh = false;
        ROS_WARN_THROTTLE(1.0, "Could NOT transform base_link to tag_1: %s", ex.what());
    }
}

void printRobotPoseLoop(tf2_ros::Buffer *tfBuffer)
{
    ros::Rate rate(10); 
    while (ros::ok() && g_tf_thread_running.load())
    {
        printRobotPose(*tfBuffer);
        rate.sleep();
    }
}

int shutdownFlow(std::thread &tf_thread, int code)
{
    g_tf_thread_running.store(false);
    ros::shutdown();
    if (tf_thread.joinable())
    {
        tf_thread.join();
    }
    return code;
}

bool hasFreshTag()
{
    if (!g_tag_fresh)
    {
        return false;
    }

    if (!g_last_tag_stamp.isZero() && (ros::Time::now() - g_last_tag_stamp).toSec() > g_tag_max_age)
    {
        g_tag_fresh = false;
        return false;
    }

    return true;
}

bool shouldRetryGrab()
{
    const ros::Time deadline = ros::Time::now() + ros::Duration(0.8);
    while (ros::ok() && ros::Time::now() < deadline)
    {
        if (hasFreshTag())
        {
            break;
        }
        ros::Duration(0.1).sleep();
    }

    if (!hasFreshTag())
    {
        if (g_last_grab_done)
        {
            grab_flag = 1;
            ROS_INFO("No fresh tag_1 TF after arm_grab; assume the object has been grabbed.");
        }
        else
        {
            ROS_WARN("No fresh tag_1 TF after a failed arm_grab; skip retry and keep grab marked failed.");
        }
        return false;
    }

    return (grab_flag == 0 && re_grab_count <= 4 && tag_x != 0.0);
}

// 临时清理代价地图

bool setCostmapLayerEnabled(ros::NodeHandle &nh, const std::string &service_name, bool enabled)
{
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter enabled_param;
    enabled_param.name = "enabled";
    enabled_param.value = enabled;
    srv.request.config.bools.push_back(enabled_param);

    ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(service_name);
    if (!client.waitForExistence(ros::Duration(0.3)))
    {
        ROS_DEBUG("Dynamic reconfigure service not available: %s", service_name.c_str());
        return false;
    }

    if (!client.call(srv))
    {
        ROS_WARN("Failed to set costmap layer: %s", service_name.c_str());
        return false;
    }

    return true;
}

void clearMoveBaseCostmaps(ros::NodeHandle &nh)
{
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty empty_srv;
    if (clear_client.waitForExistence(ros::Duration(1.0)))
    {
        clear_client.call(empty_srv);
    }
}

void setLocalCostmapBlindMode(ros::NodeHandle &nh, bool blind_mode)
{
    // Temporarily remove local-costmap constraints when this leg is known to be clear.
    const std::string services[] = {
        "/move_base/local_costmap/obstacle_layer/set_parameters",
        "/move_base/local_costmap/static_layer/set_parameters",
        "/move_base/local_costmap/costmap_prohibition_layer/set_parameters",
        "/move_base/local_costmap/inflation_layer/set_parameters",
    };

    for (const auto &service : services)
    {
        setCostmapLayerEnabled(nh, service, !blind_mode);
    }

    clearMoveBaseCostmaps(nh);
}

double normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

void publishStop()
{
    if (g_cmd_vel_pub == nullptr)
    {
        return;
    }

    geometry_msgs::Twist stop_msg;
    ros::Rate rate(20);
    for (int i = 0; ros::ok() && i < 3; ++i)
    {
        g_cmd_vel_pub->publish(stop_msg);
        rate.sleep();
    }
}

bool getBasePose(double &x, double &y, double &yaw)
{
    if (g_tf_buffer == nullptr)
    {
        ROS_ERROR("Quick dump failed: TF buffer is not initialized.");
        return false;
    }

    const std::string frames[] = {"odom", "odom_combined", "map"};
    for (const auto &frame : frames)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped =
                g_tf_buffer->lookupTransform(frame, "base_link", ros::Time(0), ros::Duration(0.2));

            x = transformStamped.transform.translation.x;
            y = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_DEBUG("Quick dump pose lookup failed in %s frame: %s", frame.c_str(), ex.what());
        }
    }

    ROS_ERROR("Quick dump failed: cannot read base_link pose from odom/odom_combined/map TF.");
    publishStop();
    return false;
}

bool rotateBaseClosedLoop(double turn_angle_deg)
{
    if (g_cmd_vel_pub != nullptr)
    {
        const ros::Time deadline = ros::Time::now() + ros::Duration(2.0);
        while (ros::ok() && g_cmd_vel_pub->getNumSubscribers() == 0 && ros::Time::now() < deadline)
        {
            ros::Duration(0.05).sleep();
        }

        if (g_cmd_vel_pub->getNumSubscribers() == 0)
        {
            ROS_ERROR("Quick dump rotate aborted: /cmd_vel has no subscribers.");
            publishStop();
            return false;
        }

        ROS_INFO("Quick dump /cmd_vel subscribers: %u", g_cmd_vel_pub->getNumSubscribers());
    }

    double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;
    if (!getBasePose(start_x, start_y, start_yaw))
    {
        return false;
    }

    const double turn_sign = (turn_angle_deg >= 0.0) ? 1.0 : -1.0;
    const double requested_turn = std::fabs(turn_angle_deg) * M_PI / 180.0;
    const double target_yaw = normalizeAngle(start_yaw + turn_sign * requested_turn);
    const double tolerance = std::max(0.1, g_quick_dump_rotate_tolerance_deg) * M_PI / 180.0;
    const double overshoot_tolerance = std::max(0.0, g_quick_dump_rotate_overshoot_tolerance_deg) * M_PI / 180.0;
    const double min_success_turn = std::max(0.0, requested_turn - tolerance);
    const double max_success_turn = requested_turn + overshoot_tolerance;
    const double max_w = std::max(0.05, std::fabs(g_quick_dump_angular_vel));
    const double min_w = std::min(max_w, std::max(0.05, std::fabs(g_quick_dump_min_angular_vel)));
    const double fine_min_w = std::min(max_w, std::max(min_w, std::fabs(g_quick_dump_fine_min_angular_vel)));
    const double slowdown_angle = std::max(3.0, g_quick_dump_slowdown_angle_deg) * M_PI / 180.0;
    const double fast_time = requested_turn / max_w;
    const double slow_time = std::min(requested_turn, slowdown_angle) / min_w;
    const double timeout = fast_time + slow_time + g_quick_dump_rotate_timeout_margin_sec;
    const double fine_tune_timeout = std::max(3.0, g_quick_dump_fine_tune_timeout_sec);
    const ros::Time begin_time = ros::Time::now();
    ros::Time fine_tune_begin_time;
    bool fine_tuning = false;
    bool overshot_correction = false;

    ROS_INFO("Quick dump rotate target: start_yaw=%.2f deg, target_yaw=%.2f deg, success_window=%.2f..%.2f deg, timeout=%.1f s",
             start_yaw * 180.0 / M_PI,
             target_yaw * 180.0 / M_PI,
             min_success_turn * 180.0 / M_PI,
             max_success_turn * 180.0 / M_PI,
             timeout);

    ros::Rate rate(30);
    while (ros::ok())
    {
        double x = 0.0, y = 0.0, yaw = 0.0;
        if (!getBasePose(x, y, yaw))
        {
            return false;
        }

        const double progress = turn_sign * normalizeAngle(yaw - start_yaw);
        const double remaining_to_target = requested_turn - progress;
        if (progress > max_success_turn && !overshot_correction)
        {
            fine_tuning = true;
            overshot_correction = true;
            fine_tune_begin_time = ros::Time::now();
            ROS_WARN("Quick dump rotate overshot success window: progress=%.2f deg, start fine correction.",
                     progress * 180.0 / M_PI);
        }

        if (progress >= min_success_turn && progress <= max_success_turn)
        {
            publishStop();
            ros::Duration(0.15).sleep();

            double settled_x = 0.0, settled_y = 0.0, settled_yaw = 0.0;
            if (!getBasePose(settled_x, settled_y, settled_yaw))
            {
                return false;
            }

            const double settled_progress = turn_sign * normalizeAngle(settled_yaw - start_yaw);
            if (settled_progress < min_success_turn || settled_progress > max_success_turn)
            {
                ROS_INFO("Quick dump rotate settling adjust: progress=%.2f deg, target_error=%.2f deg",
                         settled_progress * 180.0 / M_PI,
                         (requested_turn - settled_progress) * 180.0 / M_PI);
                continue;
            }

            ROS_INFO("Quick dump rotate reached: requested=%.2f deg, progress=%.2f deg, target_error=%.2f deg",
                     turn_angle_deg,
                     settled_progress * 180.0 / M_PI,
                     (requested_turn - settled_progress) * 180.0 / M_PI);
            return true;
        }

        if (!fine_tuning && (ros::Time::now() - begin_time).toSec() > timeout)
        {
            if (std::fabs(remaining_to_target) < slowdown_angle)
            {
                fine_tuning = true;
                fine_tune_begin_time = ros::Time::now();
                ROS_WARN("Quick dump rotate timeout near target: progress=%.2f deg, target_error=%.2f deg; keep approaching.",
                         progress * 180.0 / M_PI,
                         remaining_to_target * 180.0 / M_PI);
            }
            else
            {
                ROS_ERROR("Quick dump rotate timeout: target=%.2f deg, progress=%.2f deg, target_error=%.2f deg",
                          turn_angle_deg,
                          progress * 180.0 / M_PI,
                          remaining_to_target * 180.0 / M_PI);
                publishStop();
                return false;
            }
        }

        if (fine_tuning && (ros::Time::now() - fine_tune_begin_time).toSec() > fine_tune_timeout)
        {
            ROS_ERROR("Quick dump rotate fine correction timeout: target=%.2f deg, progress=%.2f deg, target_error=%.2f deg",
                      turn_angle_deg,
                      progress * 180.0 / M_PI,
                      remaining_to_target * 180.0 / M_PI);
            publishStop();
            return false;
        }

        if (progress > max_success_turn)
        {
            geometry_msgs::Twist correction_msg;
            correction_msg.angular.z = -turn_sign * fine_min_w;
            const ros::Time pulse_end = ros::Time::now() + ros::Duration(std::max(0.03, g_quick_dump_fine_pulse_sec));
            while (ros::ok() && ros::Time::now() < pulse_end)
            {
                g_cmd_vel_pub->publish(correction_msg);
                rate.sleep();
            }
            publishStop();
            ros::Duration(std::max(0.05, g_quick_dump_fine_settle_sec)).sleep();
            ROS_INFO_THROTTLE(1.0, "Quick dump rotate pulse correction: progress=%.2f deg, target_error=%.2f deg",
                              progress * 180.0 / M_PI,
                              remaining_to_target * 180.0 / M_PI);
            continue;
        }

        geometry_msgs::Twist vel_msg;
        double slow_w = max_w;
        if (std::fabs(remaining_to_target) < slowdown_angle)
        {
            slow_w = std::max(min_w, std::min(max_w, std::fabs(remaining_to_target) * 1.0));
        }
        vel_msg.angular.z = turn_sign * slow_w;
        g_cmd_vel_pub->publish(vel_msg);
        rate.sleep();
    }

    publishStop();
    return false;
}

bool driveForwardClosedLoop(double forward_dist_m)
{
    double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;
    if (!getBasePose(start_x, start_y, start_yaw))
    {
        return false;
    }

    const double target_dist = std::fabs(forward_dist_m);
    const double tolerance = 0.02;
    const double max_v = std::max(0.03, std::fabs(g_quick_dump_linear_vel));
    const double timeout = target_dist / max_v + 8.0;
    const ros::Time begin_time = ros::Time::now();
    const double sign = (forward_dist_m >= 0.0) ? 1.0 : -1.0;

    ros::Rate rate(30);
    while (ros::ok())
    {
        double x = 0.0, y = 0.0, yaw = 0.0;
        if (!getBasePose(x, y, yaw))
        {
            return false;
        }

        const double dx = x - start_x;
        const double dy = y - start_y;
        const double moved = std::sqrt(dx * dx + dy * dy);
        if (moved + tolerance >= target_dist)
        {
            publishStop();
            ROS_INFO("Quick dump forward reached: target=%.3f m, moved=%.3f m", forward_dist_m, moved);
            return true;
        }

        if ((ros::Time::now() - begin_time).toSec() > timeout)
        {
            ROS_ERROR("Quick dump forward timeout: target=%.3f m, moved=%.3f m", forward_dist_m, moved);
            publishStop();
            return false;
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = sign * max_v;
        g_cmd_vel_pub->publish(vel_msg);
        rate.sleep();
    }

    publishStop();
    return false;
}

void compensateArmYawForBaseTurn(double turn_angle_deg)
{
    if (g_single_servo_pub == nullptr)
    {
        ROS_WARN("Quick dump arm yaw compensation skipped: /single_servo_topic publisher is not initialized.");
        return;
    }

    // ArmPosition only exposes x/y/z. Use the existing base-joint servo topic for the minimal opposite yaw
    // compensation, where the current project uses about 10 servo ticks per degree.
    upros_message::SingleServo servo_msg;
    servo_msg.ID = 1;
    servo_msg.Target_position_Angle = static_cast<int16_t>(-turn_angle_deg * 10.0);
    servo_msg.Rotation_Speed = 120;
    g_single_servo_pub->publish(servo_msg);
    ros::Duration(0.4).sleep();
}

bool releaseObject()
{
    if (g_arm_release_client == nullptr)
    {
        ROS_ERROR("Quick dump release failed: release service client is not initialized.");
        publishStop();
        return false;
    }

    std_srvs::Empty empty_srv;
    if (!g_arm_release_client->waitForExistence(ros::Duration(1.0)))
    {
        ROS_ERROR("Quick dump release failed: /upros_arm_control/release_service is not available.");
        publishStop();
        return false;
    }

    const int release_repeat = std::max(1, g_quick_dump_release_repeat);
    for (int i = 0; i < release_repeat; ++i)
    {
        if (!g_arm_release_client->call(empty_srv))
        {
            ROS_ERROR("Quick dump release failed: /upros_arm_control/release_service call returned false on attempt %d/%d.",
                      i + 1, release_repeat);
            publishStop();
            return false;
        }

        if (i + 1 < release_repeat)
        {
            ros::Duration(std::max(0.0, g_quick_dump_release_pause_sec)).sleep();
        }
    }

    ROS_INFO("Quick dump release succeeded after %d command(s).", release_repeat);
    g_last_grab_done = false;
    ros::Duration(0.6).sleep();

    if (g_arm_zero_client == nullptr)
    {
        ROS_WARN("Quick dump zero pose skipped: zero service client is not initialized.");
    }
    else if (!g_arm_zero_client->waitForExistence(ros::Duration(1.0)))
    {
        ROS_WARN("Quick dump zero pose skipped: /upros_arm_control/zero_service is not available.");
    }
    else if (!g_arm_zero_client->call(empty_srv))
    {
        ROS_WARN("Quick dump zero pose failed: /upros_arm_control/zero_service call returned false.");
    }
    else
    {
        ROS_INFO("Quick dump arm returned to zero pose.");
        ros::Duration(0.6).sleep();
    }

    publishStop();
    return true;
}

bool runArmGrabLaunch()
{
    const int ret = std::system("roslaunch clean_desktop_robot arm_grab.launch");
    g_last_grab_done = (ret == 0);
    if (!g_last_grab_done)
    {
        ROS_ERROR("arm_grab.launch failed, return code: %d", ret);
    }
    return g_last_grab_done;
}

bool runArmPutLaunch()
{
    const int ret = std::system("roslaunch clean_desktop_robot arm_put.launch");
    if (ret != 0)
    {
        ROS_ERROR("arm_put.launch failed, return code: %d", ret);
        return false;
    }
    g_last_grab_done = false;
    return true;
}

bool quickDumpAfterGrab(double turn_angle_deg, double forward_dist_m)
{
    if (g_cmd_vel_pub == nullptr)
    {
        ROS_ERROR("Quick dump aborted: /cmd_vel publisher is not initialized.");
        return false;
    }

    if (!g_last_grab_done)
    {
        ROS_ERROR("Quick dump aborted: last grab action has not completed successfully.");
        publishStop();
        return false;
    }

    ROS_INFO("Quick dump start: flat arm hold pose is handled inside arm_grab, turn %.2f deg, forward %.3f m.",
             turn_angle_deg, forward_dist_m);

    if (!rotateBaseClosedLoop(turn_angle_deg))
    {
        ROS_ERROR("Quick dump failed while rotating base.");
        return false;
    }

    if (g_quick_dump_arm_yaw_compensation)
    {
        compensateArmYawForBaseTurn(turn_angle_deg);
    }

    if (!driveForwardClosedLoop(forward_dist_m))
    {
        ROS_ERROR("Quick dump failed while driving forward.");
        return false;
    }

    if (!releaseObject())
    {
        ROS_ERROR("Quick dump failed while releasing object.");
        return false;
    }

    publishStop();
    ROS_INFO("Quick dump finished successfully.");
    return true;
}

// 主函数初始化

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    // TF listener setup
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    std::thread tf_thread(printRobotPoseLoop, &tfBuffer);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher single_servo_pub = nh.advertise<upros_message::SingleServo>("/single_servo_topic", 10);
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // Parameters
    double grab_1_x = 1.90, grab_1_y = -1.83;
    double grab_2_x = 1.90, grab_2_y = -3.05;
    double offset_left = 0.4, offset_right = 0.4;
    bool use_quick_dump_strategy = true;
    double quick_dump_turn_angle_deg = 90.0;
    double quick_dump_forward_dist_m = 0.30;

    nh.getParam("/complete_flow_node/grab_1_x", grab_1_x);
    nh.getParam("/complete_flow_node/grab_1_y", grab_1_y);
    nh.getParam("/complete_flow_node/grab_2_x", grab_2_x);
    nh.getParam("/complete_flow_node/grab_2_y", grab_2_y);
    nh.getParam("/complete_flow_node/offset_left", offset_left);
    nh.getParam("/complete_flow_node/offset_right", offset_right);
    nh.getParam("/complete_flow_node/use_quick_dump_strategy", use_quick_dump_strategy);
    nh.getParam("/complete_flow_node/quick_dump_turn_angle_deg", quick_dump_turn_angle_deg);
    nh.getParam("/complete_flow_node/quick_dump_forward_dist_m", quick_dump_forward_dist_m);
    nh.getParam("/complete_flow_node/quick_dump_linear_vel", g_quick_dump_linear_vel);
    nh.getParam("/complete_flow_node/quick_dump_angular_vel", g_quick_dump_angular_vel);
    nh.getParam("/complete_flow_node/quick_dump_rotate_timeout_margin_sec", g_quick_dump_rotate_timeout_margin_sec);
    nh.getParam("/complete_flow_node/quick_dump_min_angular_vel", g_quick_dump_min_angular_vel);
    nh.getParam("/complete_flow_node/quick_dump_fine_min_angular_vel", g_quick_dump_fine_min_angular_vel);
    nh.getParam("/complete_flow_node/quick_dump_slowdown_angle_deg", g_quick_dump_slowdown_angle_deg);
    nh.getParam("/complete_flow_node/quick_dump_rotate_tolerance_deg", g_quick_dump_rotate_tolerance_deg);
    nh.getParam("/complete_flow_node/quick_dump_rotate_overshoot_tolerance_deg", g_quick_dump_rotate_overshoot_tolerance_deg);
    nh.getParam("/complete_flow_node/quick_dump_fine_tune_timeout_sec", g_quick_dump_fine_tune_timeout_sec);
    nh.getParam("/complete_flow_node/quick_dump_fine_pulse_sec", g_quick_dump_fine_pulse_sec);
    nh.getParam("/complete_flow_node/quick_dump_fine_settle_sec", g_quick_dump_fine_settle_sec);
    nh.getParam("/complete_flow_node/quick_dump_release_repeat", g_quick_dump_release_repeat);
    nh.getParam("/complete_flow_node/quick_dump_release_pause_sec", g_quick_dump_release_pause_sec);
    nh.getParam("/complete_flow_node/quick_dump_arm_yaw_compensation", g_quick_dump_arm_yaw_compensation);

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_close");
    upros_message::ArmPosition move_srv;

    g_cmd_vel_pub = &pub;
    g_single_servo_pub = &single_servo_pub;
    g_tf_buffer = &tfBuffer;
    g_arm_release_client = &arm_release_client;
    g_arm_zero_client = &arm_zero_client;

    move_base_msgs::MoveBaseGoal goal;
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;
    int count = 0;

    //出发区到第一张桌子

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
    g_last_grab_done = false;
    re_grab_count = 0;
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
            if (tag_x >= 0.31)  //0.31
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

        runArmGrabLaunch();

        while(ros::ok())
        {
			if (shouldRetryGrab())         //微调和重试
		{
			ROS_INFO("retry");
			if (tag_x >= 0.25)
			{
				vel_msg.linear.x = 0.1;
				count = 0;
				while (ros::ok() && count < 10)
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
			runArmGrabLaunch();
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




    //第一次投放
    bool quick_dump_1_ok = false;
    if (use_quick_dump_strategy)
    {
        quick_dump_1_ok = quickDumpAfterGrab(quick_dump_turn_angle_deg, quick_dump_forward_dist_m);
        if (!quick_dump_1_ok)
        {
            ROS_ERROR("Quick dump 1 failed; stop instead of falling back to legacy path before a confirmed 90 deg turn.");
            publishStop();
            return shutdownFlow(tf_thread, 0);
        }
    }

    if (!use_quick_dump_strategy || !quick_dump_1_ok)
    {
    // ---------------------- Backward after grab
    vel_msg.angular.z = -1.5;
    count = 0;
    while (ros::ok() && count < 9)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);


    // ---------------------- Backward after grab
    vel_msg.linear.x = 0.2;
    count = 0;
    while (ros::ok() && count < 25)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);



    // ---------------------- Backward after grab
    vel_msg.angular.z = 1.5;
    count = 0;
    while (ros::ok() && count < 12)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);




    // ---------------------- Backward after grab
    vel_msg.linear.x = 0.15;
    count = 0;
    while (ros::ok() && count < 10)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);




    runArmPutLaunch();



















    // // ---------------------- Goal 1 (Place)
    // goal.target_pose.pose.position.y = grab_1_y - offset_right;
    // goal.target_pose.header.stamp = ros::Time::now();

    // ac.sendGoal(goal);
    // ROS_INFO("MoveBase Send Goal 1 Trash !!!");
    // ac.waitForResult();

    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("Goal 1 Trash Reached!");
    //     vel_msg.linear.x = 0.1;
    //     count = 0;
    //     while (ros::ok() && count < 22)  //22
    //     {
    //         pub.publish(vel_msg);
    //         loop_rate.sleep();
    //         count++;
    //     }
    //     vel_msg.linear.x = 0.0;
    //     pub.publish(vel_msg);
    //     system("roslaunch clean_desktop_robot arm_put.launch");
    // }


    //撤离
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
    }
    
	    tag_x = 0.0;
	    tag_y = 0.0;
	    tag_yaw = 0.0;
	    grab_flag = 0;
	    ::count = 0;
	    g_tag_fresh = false;
	    g_last_tag_stamp = ros::Time(0);




    //第二张桌子
    // ---------------------- Goal 2 (Grab)
    g_last_grab_done = false;
    re_grab_count = 0;
    goal.target_pose.pose.position.x = grab_2_x;
    goal.target_pose.pose.position.y = grab_2_y;
    // Goal 2 grab heading: same as Goal 1.
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.stamp = ros::Time::now();

    setLocalCostmapBlindMode(nh, true);
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal 2 !!!");
    ac.waitForResult();
    setLocalCostmapBlindMode(nh, false);
    
    

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal 2 Reached!");

        vel_msg.linear.x = 0.07;
        count = 0;
        while (ros::ok())
        {
            if (tag_x >= 0.29)  //0.29
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
        runArmGrabLaunch();
        while (ros::ok())
        {
            if (shouldRetryGrab())
            {
                ROS_INFO("retry");
                if (tag_x >= 0.25)
                {
                    vel_msg.linear.x = 0.1;
                    count = 0;
                    while (ros::ok() && count < 10)
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
                runArmGrabLaunch();
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
        ROS_ERROR("Goal 2 failed; stop before second grab/dump.");
        publishStop();
        return shutdownFlow(tf_thread, 0);
    }





    // vel_msg.linear.x = -0.3;
    // count = 0;
    // while (ros::ok() && count < 14)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);








    //第二次投放
    bool quick_dump_2_ok = false;
    if (use_quick_dump_strategy)
    {
        quick_dump_2_ok = quickDumpAfterGrab(-quick_dump_turn_angle_deg, quick_dump_forward_dist_m);
        if (!quick_dump_2_ok)
        {
            ROS_ERROR("Quick dump 2 failed; stop instead of falling back to legacy path before a confirmed 90 deg turn.");
            publishStop();
            return shutdownFlow(tf_thread, 0);
        }
    }

    if (!use_quick_dump_strategy || !quick_dump_2_ok)
    {
    // ---------------------- Backward after grab
    vel_msg.angular.z = 1.5;
    count = 0;
    while (ros::ok() && count < 10)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);


    // ---------------------- Backward after grab
    vel_msg.linear.x = 0.2;
    count = 0;
    while (ros::ok() && count < 28)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);



    // ---------------------- Backward after grab
    vel_msg.angular.z = -1.5;
    count = 0;
    while (ros::ok() && count < 12)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);


   // ---------------------- Backward after grab
    vel_msg.linear.x = 0.15;
    count = 0;
    while (ros::ok() && count < 12)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);




    runArmPutLaunch();








    // // ---------------------- Goal 2 (Place)
    // goal.target_pose.pose.position.y = grab_2_y + offset_left;
    // goal.target_pose.header.stamp = ros::Time::now();

    // ac.sendGoal(goal);
    // ROS_INFO("MoveBase Send Goal 2 Trash !!!");
    // ac.waitForResult();

    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("Goal 2 Trash Reached!");
    //     vel_msg.linear.x = 0.1;
    //     count = 0;
    //     while (ros::ok() && count < 24)   //24
    //     {
    //         pub.publish(vel_msg);
    //         loop_rate.sleep();
    //         count++;
    //     }
    //     vel_msg.linear.x = 0.0;
    //     pub.publish(vel_msg);
    //     system("roslaunch clean_desktop_robot arm_put.launch");
    // }


    //撤离

    vel_msg.linear.x = -0.5;
    count = 0;
    while (ros::ok() && count < 20)    //50
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    }















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
    
    













    //返回出发区

    // ---------------------- Return Home 2
    // Return home: navigate to a safer point away from the wall, then drive straight into the start area.
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 0.4;
    goal.target_pose.pose.position.y = 0.4;
    goal.target_pose.pose.orientation.z = 0.93358;
    goal.target_pose.pose.orientation.w = -0.35837;
    goal.target_pose.header.stamp = ros::Time::now();

    // Return-only blind navigation: disable local laser obstacles before entering the wall-side start area.
    // setLocalCostmapBlindMode(nh, true);

    ac.sendGoal(goal);
    ROS_INFO("Send Goal Home 2 !!!");
    ac.waitForResult();

    // setLocalCostmapBlindMode(nh, false);

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back to Home 2!");
    }
    else
    {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        pub.publish(vel_msg);
        return shutdownFlow(tf_thread, 0);
    }

    // Final entry: keep the reached heading and drive straight back into the start area.
    vel_msg.linear.x = 0.3;
    vel_msg.angular.z = 0.0;
    count = 0;
    while (ros::ok() && count < 15)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    return shutdownFlow(tf_thread, 0);
}
