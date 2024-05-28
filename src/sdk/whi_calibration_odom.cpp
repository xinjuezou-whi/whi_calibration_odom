/******************************************************************
odom calibration process

Features:
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_calibration_odom/whi_calibration_odom.h"

#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>

#include <thread>

namespace whi_motion_interface
{
    CalibrationOdom::CalibrationOdom(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    CalibrationOdom::~CalibrationOdom()
    {
        if (srv_linear_)
        {
            srv_linear_->shutdown();
        }
        if (srv_angular_)
        {
            srv_angular_->shutdown();
        }
    }

    void CalibrationOdom::init()
    {
        // params
        node_handle_->getParam("linears", linears_);
        node_handle_->getParam("angulars", angulars_);
        for (auto& it : angulars_)
        {
            it = angles::from_degrees(it);
        }
        node_handle_->param("wait_span", wait_span_, 5.0);
        node_handle_->param("twist_topic", twist_topic_, std::string("cmd_vel"));
        node_handle_->param("vel_linear", vel_linear_, 0.025);
        node_handle_->param("vel_angular", vel_angular_, 0.1);
        node_handle_->param("thresh_linear", thresh_linear_, 0.005);
        node_handle_->param("thresh_angular", thresh_angular_, 2.0);
        thresh_angular_ = angles::from_degrees(thresh_angular_);
        // twist publisher
        pub_twist_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<geometry_msgs::Twist>(twist_topic_, 50));

        // providing the services
        srv_linear_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("linear", &CalibrationOdom::onServiceLinear, this));
        srv_angular_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("angular", &CalibrationOdom::onServiceAngular, this));

        // tf listening
        node_handle_->param("map_frame", map_frame_, std::string("odom"));
        node_handle_->param("base_frame", base_link_frame_, std::string("base_link"));
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);

        // spinner
        node_handle_->param("loop_hz", loop_hz_, 20.0);
    }

    static double distance(const geometry_msgs::Pose& PoseA, const geometry_msgs::Pose& PoseB)
    {
        return sqrt(pow(PoseA.position.x - PoseB.position.x, 2) + 
            pow(PoseA.position.y - PoseB.position.y, 2));
    }

    static geometry_msgs::Pose applyTransform(const geometry_msgs::Pose& Src,
        const geometry_msgs::TransformStamped& Transform)
    {
        // apply the transform to the pose
        geometry_msgs::Pose transformedPose;
        tf2::doTransform(Src, transformedPose, Transform);

        return transformedPose;
    }

    template <typename T> int signOf(T Val)
    {
	    return (T(0) < Val) - (Val < T(0));
    }

    bool CalibrationOdom::onServiceLinear(std_srvs::Trigger::Request& Req, std_srvs::Trigger::Response& Res)
    {
        for (int i = 0; i < linears_.size(); ++i)
        {
            auto currentTf = listenTf(map_frame_, base_link_frame_, ros::Time(0));
            geometry_msgs::Pose currentPos;
            currentPos.position.x = currentTf.transform.translation.x;
            currentPos.position.y = currentTf.transform.translation.y;
            currentPos.orientation.w = 1.0;
            geometry_msgs::TransformStamped trans;
            trans.transform.translation.x = linears_[i];
            trans.transform.rotation.w = 1.0;
            geometry_msgs::Pose endPos = applyTransform(currentPos, trans);
            
            geometry_msgs::Twist msg;
            while (distance(currentPos, endPos) > thresh_linear_)
            {
		        msg.linear.x = signOf(linears_[i]) * vel_linear_;
		        msg.angular.z = 0.0;
                pub_twist_->publish(msg);

                currentTf = listenTf(map_frame_, base_link_frame_, ros::Time(0));
                currentPos.position.x = currentTf.transform.translation.x;
                currentPos.position.y = currentTf.transform.translation.y;

                std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / loop_hz_)));
            }
            msg.linear.x = 0.0;
		    msg.angular.z = 0.0;
            pub_twist_->publish(msg);

            if (i < linears_.size() - 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_span_ * 1000)));
            }   
        }
        ROS_INFO_STREAM("linear sequence are all traversed");

        Res.success = true;
        return Res.success;
    }

    static std::array<double, 3> toEuler(const geometry_msgs::Quaternion& Orientation)
    {
        tf2::Quaternion quaternion(Orientation.x, Orientation.y, Orientation.z, Orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        return { roll, pitch, yaw };
    }

    bool CalibrationOdom::onServiceAngular(std_srvs::Trigger::Request& Req, std_srvs::Trigger::Response& Res)
    {
        for (int i = 0; i < angulars_.size(); ++i)
        {
            auto currentTf = listenTf(map_frame_, base_link_frame_, ros::Time(0));
            geometry_msgs::Pose currentPos;
            currentPos.orientation = currentTf.transform.rotation;
            auto currentEuler = toEuler(currentPos.orientation);
            double endYaw = currentEuler[2] + angulars_[i];

            geometry_msgs::Twist msg;
            while (fabs(currentEuler[2] - endYaw) > thresh_angular_)
            {
		        msg.linear.x = 0.0;
		        msg.angular.z = signOf(angulars_[i]) * vel_angular_;
                pub_twist_->publish(msg);

                currentTf = listenTf(map_frame_, base_link_frame_, ros::Time(0));
                currentPos.orientation = currentTf.transform.rotation;
                currentEuler = toEuler(currentPos.orientation);

                std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / loop_hz_)));
            }
            msg.linear.x = 0.0;
		    msg.angular.z = 0.0;
            pub_twist_->publish(msg);

            if (i < angulars_.size() - 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_span_ * 1000)));
            }
        }
        ROS_INFO_STREAM("angular sequence are all traversed");

        Res.success = true;
        return Res.success;
    }

    geometry_msgs::TransformStamped CalibrationOdom::listenTf(const std::string& DstFrame, const std::string& SrcFrame,
        const ros::Time& Time)
    {
        try
        {
            if (buffer_.canTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0)))
            {
                return buffer_.lookupTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0));
            }
            else
            {
				auto pose = geometry_msgs::TransformStamped();
				pose.transform.rotation.w = 1.0;
                return pose;
            }
        }
        catch (tf2::TransformException &e)
        {
            ROS_ERROR("%s", e.what());

			auto pose = geometry_msgs::TransformStamped();
			pose.transform.rotation.w = 1.0;
            return pose;
        }
    }
}
