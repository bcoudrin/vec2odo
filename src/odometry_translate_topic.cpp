#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "vec2odo/xyzrpy2odo.h"
#include "vec2odo/vector6.h"

#include <sstream>

nav_msgs::Odometry odo_glob;
tf::TransformBroadcaster *br;

void odometryCallback(const vec2odo::vector6ConstPtr& vec)
{
  odo_glob.header.stamp = ros::Time::now();
  odo_glob.header.frame_id = "odom";

  // Pose center
  odo_glob.pose.pose.position.x = vec->x;
  odo_glob.pose.pose.position.y = vec->y;
  odo_glob.pose.pose.position.z = vec->z;

  // Pose attitute
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(vec->ro, vec->pi, vec->ya);
  odo_glob.pose.pose.orientation = odom_quat;

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(vec->x, vec->y, vec->z));
  transform.setRotation(tf::Quaternion(vec->ro, vec->pi, vec->ya));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

  // Twist
  odo_glob.child_frame_id = "base_link";
  odo_glob.twist.twist.linear.x = 0.0;
  odo_glob.twist.twist.linear.y = 0.0;
  odo_glob.twist.twist.linear.y = 0.0;
  odo_glob.twist.twist.angular.x = 0.0;
  odo_glob.twist.twist.angular.y = 0.0;
  odo_glob.twist.twist.angular.z = 0.0;

  ROS_INFO("Request :\n   xyz = [%f, %f, %f]\n   rpy = [%f, %f, %f]", vec->x, vec->y, vec->z, vec->ro, vec->pi, vec->ya);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_translate");
  ros::NodeHandle n;

  odo_glob = nav_msgs::Odometry();

  tf::Transform transform;
  br = new tf::TransformBroadcaster();

  ros::Subscriber sub = n.subscribe("state_vector_stream", 1000, odometryCallback);
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("odometry_stream", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0.0, 0.0, 0.0) );
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
    chatter_pub.publish(odo_glob);
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete br;

  return 0;
}
