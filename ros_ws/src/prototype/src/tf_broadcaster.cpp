#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.00, 0.0, 0.00)),
        ros::Time::now(),"/base_link", "/odom"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.095, 0.0, 0.05)),
        ros::Time::now(),"/base_link", "/camera_link"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.00, 0.0, 0.05)),
        ros::Time::now(),"/base_link", "/base_footprint"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, 0.0, 0.105)),
        ros::Time::now(),"/base_link", "/imu"));
      broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, 0.0, 0.105)),
        ros::Time::now(),"/base_link", "/imu_link"));
      broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.02, 0.0, 0.105)),
        ros::Time::now(),"/base_link", "/imu_link_ned"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.105)),
        ros::Time::now(),"/base_link", "/fix"));
    r.sleep();
  } 
}