#include "wheel_odometer_integrator/wheel_odometer_integrator.hpp"

namespace perception{
WheelOdometerRepub::WheelOdometerRepub(): Node("wheel_odometer")
{
  odomFrame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  // Changing this frame is strongly discouraged.
  selfFrame_ = this->declare_parameter<std::string>("base_frame", "rear_axle_middle");

  wheelSpdRptTopic_ = this->declare_parameter<std::string>("wheel_speed_report", "wheel_speed_report");
  steerRptTopic_ = this->declare_parameter<std::string>("steering_report", "steering_report");
  odomTopic_ = this->declare_parameter<std::string>("output_odom_topic", "odometry");
  spinMonitorTopic_ = this->declare_parameter<std::string>("output_spin_topic", "spin_monitor_test");
  

  this->noSlipYawRate_ = 0.;
	this->steeringAngle_ = 0.;
	this->velocity_ = 0.;
	this->odomMsg_.header.frame_id=odomFrame_;
	this->odomMsg_.child_frame_id=selfFrame_;
  this->odomMsg_.pose.pose.position.x = 0.0;
  this->odomMsg_.pose.pose.position.y = 0.0;
  this->odomMsg_.pose.pose.position.z = 0.0;
  this->odomMsg_.pose.pose.orientation.x = 0.0;
  this->odomMsg_.pose.pose.orientation.y = 0.0;
  this->odomMsg_.pose.pose.orientation.z = 0.0;
  this->odomMsg_.pose.pose.orientation.w = 1.0;
  this->odomMsg_.pose.covariance.fill(0.0);
  this->odomMsg_.twist.twist.linear.x = 0.0;
  this->odomMsg_.twist.twist.linear.y = 0.0;
  this->odomMsg_.twist.twist.linear.z = 0.0;
  this->odomMsg_.twist.twist.angular.x = 0.0;
  this->odomMsg_.twist.twist.angular.y = 0.0;
  this->odomMsg_.twist.twist.angular.z = 0.0;
  this->odomMsg_.twist.covariance.fill(0.0);

	this->odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      odomTopic_, rclcpp::SystemDefaultsQoS());
  this->spinMonPub_ = this->create_publisher<std_msgs::msg::Float32>(
      spinMonitorTopic_, rclcpp::SystemDefaultsQoS());
  this->subSteering_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
	    steerRptTopic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelOdometerRepub::receiveSteer, this, std::placeholders::_1));
	this->subWheelspeed_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
	    wheelSpdRptTopic_, rclcpp::SensorDataQoS(),
      std::bind(&WheelOdometerRepub::receiveWheelSpeeds, this, std::placeholders::_1));
	
  // spinning detection threshold (rad/s) --
  // comparing the difference of yaw rate computed from steering and from wheel speed difference.
  // if the discripency between the two values exceed this threshold, a spin has occured.
  // TODO: needs testing.
  spinDetectThr_ = this->declare_parameter<double>("spinning_detection_threshold", 0.3);
  spinMonAlpha_ = this->declare_parameter<double>("spin_exponential_filter_const", 0.5);
  spinMonAvg_ = 0.0;

  trkWidth_ = this->declare_parameter<double>("vehicle.track_width", 1.5813);
	whlBase_ = this->declare_parameter<double>("vehicle.wheel_base", 2.9718);
	spdCov_ = this->declare_parameter<double>("vehicle.odom_speed_covariance", 1.0);
  posCov_ = this->declare_parameter<double>("vehicle.odom_position_covariance", 1.0);
  strCov_ = this->declare_parameter<double>("vehicle.steering_angle_covariance", 0.1);

  // TODO: make sure of this -- deg2rad / steerAngleBehindTheWheels
  // TODO: maybe the sign should be inverted -- left steer should yield a positive value.
  double steeringGearRatio = this->declare_parameter<double>("vehicle.steering.gear_ratio", 15.);
  strRptIU2SI_ = M_PI/180./steeringGearRatio;

  paramCbHandle_ = this->add_on_set_parameters_callback(
    std::bind(&WheelOdometerRepub::parametersCallback, this, std::placeholders::_1));
}

void WheelOdometerRepub::receiveWheelSpeeds(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg){
  // No conversion is needed.
  double tfSteer = tanf(steeringAngle_);
  double leftSteerAngle = atan(tfSteer/(1. - trkWidth_*0.5*tfSteer/whlBase_));
  double rightSteerAngle = atan(tfSteer/(1. + trkWidth_*0.5*tfSteer/whlBase_));

  // front left and front right wheels has larger turning radius when steering
  // convert them to rear axle values by scaling the cosine of steering angles.
  double flWhlSpdProjToRL = msg->front_left/cosf(leftSteerAngle);
  double frWhlSpdProjToRR = msg->front_right/cosf(rightSteerAngle);
  const double KMPH2MPS = 1000./3600.;
  velocity_ = (msg->rear_left + msg->rear_right + flWhlSpdProjToRL + frWhlSpdProjToRR)*0.25 * KMPH2MPS;

  double curvature = tanf(this->steeringAngle_)/this->whlBase_;

  double yawRateFromSteer = velocity_ * curvature;
  double yawRateFromVelDiff = (msg->rear_right - msg->rear_left + frWhlSpdProjToRR - flWhlSpdProjToRL)*0.5 * KMPH2MPS / trkWidth_;

  noSlipYawRate_ = (yawRateFromSteer + yawRateFromVelDiff)*0.5;

  double oldStamp = odomMsg_.header.stamp.sec + odomMsg_.header.stamp.nanosec * 1.0e-9;
  odomMsg_.header.stamp = msg->header.stamp;
  double dt = odomMsg_.header.stamp.sec + odomMsg_.header.stamp.nanosec * 1.0e-9 - oldStamp;
  // integrate the change
  tf2::Vector3 v;
  tf2::fromMsg(odomMsg_.twist.twist.linear, v);
  tf2::Vector3 v_new(this->velocity_, 0., 0.);
  tf2::Vector3 v_ang;
  tf2::fromMsg(odomMsg_.twist.twist.angular, v_ang);
  tf2::Vector3 v_ang_new(0., 0., this->noSlipYawRate_);
  tf2::Quaternion q;
  tf2::fromMsg(odomMsg_.pose.pose.orientation, q);
  tf2::Vector3 p;
  tf2::fromMsg(odomMsg_.pose.pose.position, p);
  // perform integration
  // advance v by dt/2
  v += 0.5 * (v_new - v);
  // advance v_ang by dt/2
  v_ang += 0.5 * (v_ang_new - v_ang);
  // advance p by dt/2
  p += tf2::quatRotate(q, v) * dt * 0.5;
  // advance q by dt/2
  tf2::Quaternion dq;
  dq.setRPY(v_ang.x()*dt*0.5, v_ang.y()*dt*0.5, v_ang.z()*dt*0.5);
  q = (q*dq).normalize();
  // rotate v to new q and advance p by another dt/2
  p += tf2::quatRotate(q, v) * dt * 0.5;
  // advance q by another dt/2
  q = (q*dq).normalize();
  odomMsg_.header.stamp = this->get_clock()->now();
  // advance v by dt/2 and advance v_ang by dt/2
  odomMsg_.twist.twist.linear = tf2::toMsg(v_new);
  odomMsg_.twist.twist.angular = tf2::toMsg(v_ang_new);
  // difference between two calculated yaw rates
  std_msgs::msg::Float32 spinMsg;
  double spin = yawRateFromVelDiff - yawRateFromSteer;
  spinMonAvg_ = spinMonAlpha_ * spin + (1-spinMonAlpha_)*spinMonAvg_;
  spinMsg.data = spinMonAvg_;
  odomMsg_.pose.pose.position.x = p.x();
  odomMsg_.pose.pose.position.y = p.y();
  odomMsg_.pose.pose.position.z = p.z();
  odomMsg_.pose.pose.orientation = tf2::toMsg(q);
  odomMsg_.twist.covariance[0] = spdCov_;     // cov(Wheel speed * r_w)
	odomMsg_.twist.covariance[7] = spdCov_;   // Y speed
  odomMsg_.twist.covariance[14] = 1.e6;  // Z speed
  odomMsg_.twist.covariance[21] = 1.e6;  // Roll speed
  odomMsg_.twist.covariance[28] = 1.e6;  // Pitch speed
  odomMsg_.twist.covariance[35] = 1.0e-3 + spdCov_*curvature*curvature +
                                        velocity_*velocity_*steerAngCov_/(whlBase_*whlBase_) + 
                                        spinMsg.data * spinMsg.data * (M_PI*0.25);
  odomMsg_.pose.covariance[0] = posCov_;   // x position covariance
  odomMsg_.pose.covariance[7] = posCov_;   // y position covariance
  odomMsg_.pose.covariance[14] = 1.e6;  // z position covariance
  odomMsg_.pose.covariance[21] = 1.e6;  // Roll angle covariance
  odomMsg_.pose.covariance[28] = 1.e6;  // Pitch angle covariance
  odomMsg_.pose.covariance[35] = 1.0e-3 + posCov_*curvature*curvature + 
                                (velocity_*velocity_*steerAngCov_/(whlBase_*whlBase_) +
                                spinMsg.data * spinMsg.data * (M_PI*0.25)) * dt * dt;  // yaw angle covariance

  if (fabs(spinMsg.data) > spinDetectThr_){
    // when spinning, the wheel speeds will not represent the actual motion of the car.
    // Enlarge covariance by 1000 times.
    odomMsg_.twist.covariance[0] *= 1000.;
    odomMsg_.twist.covariance[7] *= 1000.;
    odomMsg_.twist.covariance[35] *= 1000.;
    odomMsg_.pose.covariance[0] *= 1000.;
    odomMsg_.pose.covariance[7] *= 1000.;
    odomMsg_.pose.covariance[35] *= 1000.;
  }
  spinMonPub_->publish(spinMsg);
  odomPub_->publish(odomMsg_);
}

void WheelOdometerRepub::receiveSteer(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg){
  //TODO: Check which field to copy from...
  this->steeringAngle_ = msg->steering_wheel_angle * this->strRptIU2SI_;
}

rcl_interfaces::msg::SetParametersResult
WheelOdometerRepub::parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
  rcl_interfaces::msg::SetParametersResult result;
  bool success = false;
  for (const auto& param : parameters){
    if (param.get_name() == "spinning_detection_threshold"){
      spinDetectThr_ = param.as_double();
      success = true;
    } else if (param.get_name() == "vehicle.steering_report_angle_IU_to_SI_gain"){
      strRptIU2SI_ = param.as_double();
      success = true;
    } else if (param.get_name() == "vehicle.wheel_speed_covariance"){
      wheelSpeedCov_ = param.as_double();
      success = true;
    } else if (param.get_name() == "spin_exponential_filter_const") {
      spinMonAlpha_ = param.as_double();
      success = true;
    }
  }
  result.successful = success;
  if (success)
    result.reason = "Success.";
  else
    result.reason = "Parameter is not supported for online updating.";
  return result;
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::WheelOdometerRepub>());
  rclcpp::shutdown();
  return 0;
}
