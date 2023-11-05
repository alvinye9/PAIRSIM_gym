#ifndef _WHEEL_ODOM_REPUB_H_
#define _WHEEL_ODOM_REPUB_H_

#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "raptor_dbw_msgs/msg/steering_report.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2
{
inline
geometry_msgs::msg::Vector3 toMsg(const Vector3 & in)
{
  geometry_msgs::msg::Vector3 out;
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

inline
void fromMsg(const geometry_msgs::msg::Vector3 & in, Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

inline
void fromMsg(const geometry_msgs::msg::Point & in, Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}
}
#endif

namespace perception{
	class WheelOdometerRepub: public rclcpp::Node
	{
	public:
		WheelOdometerRepub();

	private:
		void receiveWheelSpeeds(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
		void receiveSteer(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg);
		rcl_interfaces::msg::SetParametersResult parametersCallback(
			const std::vector<rclcpp::Parameter> &parameters);

    	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spinMonPub_;
		rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr subSteering_;
		rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr subWheelspeed_;
		rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr paramCbHandle_;
		
		rclcpp::Time steer_time_;
        double spinMonAvg_;

		std::string odomFrame_, selfFrame_, wheelSpdRptTopic_, steerRptTopic_, odomTopic_;
		std::string spinMonitorTopic_;
		double trkWidth_, whlBase_, wheelRadius_, spdCov_, strCov_, posCov_, strRptIU2SI_, spinDetectThr_, spinMonAlpha_;
		double steeringAngle_, noSlipYawRate_, velocity_;
		double wheelSpeedCov_, steerAngCov_;
		nav_msgs::msg::Odometry odomMsg_;
	};
}
#endif
