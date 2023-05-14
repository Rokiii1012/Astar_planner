#include<init_pose_subscriber.h>


InitPoseSubscriber::InitPoseSubscriber(ros::NodeHandle &nh,const std::string &topic_name,size_t buffer ) {

  sub_ = nh.subscribe(topic_name,buffer,&InitPoseSubscriber::MessageCallBack,this);

}
// 消息加上const
void InitPoseSubscriber::MessageCallBack(const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_msg) {

  init_pose_ = init_pose_msg; 
  ROS_WARN("Init_pose_x:%2f",init_pose_->pose.pose.position.x);
  ROS_WARN("Init_pose_y:%2f",init_pose_->pose.pose.position.y);
  ROS_WARN("Init_pose_z:%2f",init_pose_->pose.pose.position.z);


}