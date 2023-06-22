#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_follower");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

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

    // Read
    double angle = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    ROS_WARN("GO");
    ROS_WARN_STREAM("ANGLE: " << angle);
    ROS_WARN_STREAM("LINEAR X: " << x);
    ROS_WARN_STREAM("LINEAR Y: " << y);

    // Send commands
    geometry_msgs::Twist cmd;

    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;

    pub.publish(cmd);

    rate.sleep();
  }
  return 0;
};