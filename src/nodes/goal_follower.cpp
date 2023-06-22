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

    // Send commands
    double angular = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    double linear = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    geometry_msgs::Twist cmd;

    if (linear > 0.2)
    {
        cmd.angular.z = std::max(std::min(angular, 0.5), -0.5);

        // Only move forward if little to no angular velocity
        if (std::abs(cmd.angular.z) < 0.35)
            cmd.linear.x = std::min(0.15, linear);
        else
            cmd.linear.x = 0.0;

        pub.publish(cmd);
        ROS_WARN("GO");
        ROS_WARN_STREAM("ANGULAR: " << angular);
        ROS_WARN_STREAM("LINEAR: " << linear);
    }
    else
    {
        // Empty command
        pub.publish(cmd);
        ROS_ERROR("STOP");
        ROS_ERROR_STREAM("ANGULAR: " << angular);
        ROS_ERROR_STREAM("LINEAR: " << linear);
    }

    rate.sleep();
  }
  return 0;
};