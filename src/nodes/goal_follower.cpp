#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_follower");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/rig", "/goal",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    geometry_msgs::Twist msg;
    msg.linear.x = 123.;
    msg.linear.y = 123.;
    msg.angular.z = 123;

    pub.publish(msg);

    rate.sleep();
  }
  return 0;
};