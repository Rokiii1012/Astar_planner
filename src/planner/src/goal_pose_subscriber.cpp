#include "goal_pose_subscriber.h"


GoalPoseSubscriber::GoalPoseSubscriber(ros::NodeHandle &nh,const std::string &topic_name,size_t buffer) {

  sub_ = nh.subscribe(topic_name,buffer,&GoalPoseSubscriber::MessageCallBack,this);

}

void GoalPoseSubscriber::MessageCallBack(const geometry_msgs::PoseStampedPtr &goal_pose) {

  goal_pose_ = goal_pose;
  ROS_WARN("goal_pose_x:%2f",goal_pose_->pose.position.x);
  ROS_WARN("goal_pose_y:%2f",goal_pose_->pose.position.y);
  ROS_WARN("goal_pose_z:%2f",goal_pose_->pose.position.z);

  // ROS_INFO("recive_goal");


}