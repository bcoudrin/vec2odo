#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "vec2odo/xyzrpy2odo.h"

bool compile(vec2odo::xyzrpy2odo::Request  &req,
             vec2odo::xyzrpy2odo::Response &res) {
  res.odo.header.stamp = ros::Time::now();
  res.odo.header.frame_id = "odom";

  // Pose center
  res.odo.pose.pose.position.x = req.xyz.x;
  res.odo.pose.pose.position.y = req.xyz.z;
  res.odo.pose.pose.position.z = req.xyz.z;

  // Pose attitute
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(req.rpy.x, req.rpy.y, req.rpy.z);
  res.odo.pose.pose.orientation = odom_quat;

  // Twist
  res.odo.child_frame_id = "base_link";
  res.odo.twist.twist.linear.x = 0.0;
  res.odo.twist.twist.linear.y = 0.0;
  res.odo.twist.twist.linear.y = 0.0;
  res.odo.twist.twist.angular.x = 0.0;
  res.odo.twist.twist.angular.y = 0.0;
  res.odo.twist.twist.angular.z = 0.0;

  ROS_INFO("Request :\n   xyz = [%f, %f, %f]\n   rpy = [%f, %f, %f]", req.xyz.x, req.xyz.y, req.xyz.z, req.rpy.x, req.rpy.y, req.rpy.z);
  //ROS_INFO("Response :\n   xyz = [%f, %f, %f]\n   rpy = [%f, %f, %f]", req.xyz.x, req.xyz.y, req.xyz.z, req.rpy.x, req.rpy.y, req.rpy.z);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xyzrpy2odometry");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("odometry_translate", compile);
  ROS_INFO("Ready to convert xyzrpy to Odometry.");
  ros::spin();

  return 0;
}
