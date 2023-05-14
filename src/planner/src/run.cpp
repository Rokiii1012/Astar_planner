#include <ros/ros.h>

#include "astar_planner.h"
#include "init_pose_subscriber.h"
#include "goal_pose_subscriber.h"
#include "nav_msgs/Path.h"
#include "backward.hpp"

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char  *argv[])
{
  
  ros::init(argc,argv,"run");
  //  
  ros::NodeHandle nh("~");

  AstarPlanner* astar_planner = new AstarPlanner(nh);
  InitPoseSubscriber* init_pose = new InitPoseSubscriber(nh,"/initialpose",1);
  GoalPoseSubscriber* goal_pose = new GoalPoseSubscriber(nh,"/move_base_simple/goal",1);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/trajectory",1,true);
  // ROS_WARN("%f",astar_planner->cost_map_.front()->info.resolution);
  // astar_planner->InitMap(astar_planner->cost_map_.front()->info.resolution,
  //                                                    astar_planner->cost_map_.front()->info.width,
  //                                                    astar_planner->cost_map_.front()->info.height);

  // astar_planner->InitMap(0.2 ,100,100);
  // for (int w = 0;w < astar_planner->cost_map_.front()->info.width;w++) {

  //   for (int h = 0;h < astar_planner->cost_map_.front()->info.height;h++) {
  //       // 0为无障碍物，100为有障碍物，-1为未知
  //       if (astar_planner->cost_map_.front()->data[h * astar_planner->cost_map_.front()->info.width + w]) {

  //         Eigen::Vector2i index = Vector2i::Zero();
  //         index(0) = w;
  //         index(1) = h;
  //         astar_planner->SetObs(index); 

  //       }


  //   }

  // }


  

  
  ros::Rate r(1);
  nav_msgs::Path search_path;
  while (ros::ok()) 
  {
    
    ros::spinOnce();

    // 假如说不判断cost_map_是否为空，可能话题传递的消息有延迟，就会导致访问越界。而且传递消息的的还是个智能指针，可能会消散，然后访问也会越界。
    if (!astar_planner->cost_map_.empty()) {
      
      if (!astar_planner->has_map_ ) {

          ROS_WARN("%d",astar_planner->cost_map_.front()->info.width);
           astar_planner->InitMap(astar_planner->cost_map_.front()->info.resolution,
                                                             astar_planner->cost_map_.front()->info.width,
                                                             astar_planner->cost_map_.front()->info.height);

        for (int w = 0;w < astar_planner->cost_map_.front()->info.width;w++) {

            for(int h = 0; h < astar_planner->cost_map_.front()->info.height;h++) {

                if (astar_planner->cost_map_.front()->data[h * astar_planner->cost_map_.front()->info.width + w]) {

                  Eigen::Vector2i index(w,h);

                  astar_planner->SetObs(index);

                }

            }

        }

      }
      
    }



    if (!astar_planner->search_success_) {

   

      if(init_pose->init_pose_  !=  nullptr && goal_pose->goal_pose_ != nullptr) {


          
          search_path.poses.clear();
          Eigen::Vector2d init_coord(init_pose->init_pose_->pose.pose.position.x,init_pose->init_pose_->pose.pose.position.y);
          Eigen::Vector2i init_index =  astar_planner->CoordToIndex(init_coord);
          Node* start_node = new Node(init_coord,init_index);
          // ROS_WARN("%2f",init_index(0));
          // ROS_WARN("%2f",init_index(1));
          // ROS_WARN("%d",init_index(0));
          // ROS_WARN("%d",init_index(1));

          Eigen::Vector2d goal_coord(goal_pose->goal_pose_->pose.position.x,goal_pose->goal_pose_->pose.position.y);
          Eigen::Vector2i goal_index =  astar_planner->CoordToIndex(goal_coord);
          Node* goal_node = new Node(goal_coord,goal_index);


      astar_planner->AstarPathFinding(start_node,goal_node);
      std::vector<Eigen::Vector2d> coord = astar_planner->GetPath();
      
      init_pose->init_pose_  = nullptr;
       goal_pose->goal_pose_ = nullptr;

      delete start_node;
      delete goal_node;

      astar_planner->search_success_ = false;

      for(auto ptr : coord) {

        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.header.stamp = ros::Time::now();
        p.pose.position.x = ptr(0);
        p.pose.position.y = ptr(1);

        search_path.poses.push_back(p);
        //  ROS_WARN("X:%2f",ptr(0));
        //  ROS_WARN("Y:%2f",ptr(1)); 
       

      }
      
    


      }
    

    }
    search_path.header.frame_id = "map";
    search_path.header.stamp = ros::Time::now();
   path_pub.publish(search_path);
   
    r.sleep();

  }


 
  return 0;
}
