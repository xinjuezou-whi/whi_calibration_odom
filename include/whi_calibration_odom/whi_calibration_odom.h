/******************************************************************
odom calibration process

Features:
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-05-28: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>

namespace whi_motion_interface
{
	class CalibrationOdom
	{
    public:
        CalibrationOdom(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~CalibrationOdom();

    protected:
        void init();
        bool onServiceLinear(std_srvs::Trigger::Request& Req, std_srvs::Trigger::Response& Res);
        bool onServiceAngular(std_srvs::Trigger::Request& Req, std_srvs::Trigger::Response& Res);
        geometry_msgs::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame,
            const ros::Time& Time);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_hz_{ 10.0 };
        std::unique_ptr<ros::Publisher> pub_twist_ = nullptr;
        std::unique_ptr<ros::ServiceServer> srv_linear_{ nullptr };
        std::unique_ptr<ros::ServiceServer> srv_angular_{ nullptr };
        std::vector<double> linears_;
        std::vector<double> angulars_;
        std::string twist_topic_{ "cmd_vel" };
        double vel_linear_{ 0.025 };
        double vel_angular_{ 0.1 };
        double thresh_linear_{ 0.005 };
        double thresh_angular_{ 2.0 };
        double wait_span_{ 5.0 };

        // tf
        tf2_ros::Buffer buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
        std::string map_frame_{ "odom" };
        std::string base_link_frame_{ "base_link" };
	};
}
