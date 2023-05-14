#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string.h>

class GoalPoseSubscriber {

  public:
      GoalPoseSubscriber(ros::NodeHandle &nh,const std::string &topic_name,size_t buffer);

      ~GoalPoseSubscriber() {};

      geometry_msgs::PoseStampedPtr goal_pose_;

  private:

   void MessageCallBack(const geometry_msgs::PoseStampedPtr &goal_pose);

      ros::Subscriber sub_;


};