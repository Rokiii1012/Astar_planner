#pragma once

#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<ros/ros.h>
#include <string>

class InitPoseSubscriber {

  public:
      explicit InitPoseSubscriber(ros::NodeHandle &nh,const std::string &topic_name,size_t buffer);

      ~ InitPoseSubscriber() {};

      geometry_msgs::PoseWithCovarianceStampedPtr init_pose_;

  private:

      void MessageCallBack(const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_message);

      ros::Subscriber sub_;


};