#pragma once

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Path.h>

#include"node.h"



class AstarPlanner {
 private:

  // 地图的尺寸X，Y
  int x_size_,y_size_;
  // 地图的分辨率
  double resolution_;
  //二维地图，Map2d[i][j]存储的是一个节点指针，为什么要存储指针，节点变量不行吗？
  Node*** Map2d_ = nullptr;
  //可排序的容器
  std::multimap<double,Node*> open_set_;
  std::vector<Eigen::Vector3d> path_;
  Node* start_node_;
  Node* target_node_;
  ros::Subscriber sub_;




 public:
  // 可以通过planner干什么
  // 按说地图的创建不该出现在这，先写着
  explicit AstarPlanner(ros::NodeHandle &nh) {

    // 想要在类内实现话题订阅，标准写法，sub_必须是一个成员变量，如果为临时变量则会构造函数消失时，sub_就会断掉
    sub_ = nh.subscribe("/map",1,&AstarPlanner::MessageCallBack,this);

  }

  ~AstarPlanner(){};

 // 这是一个回调函数，当有新消息到达/map话题时它就会被调用。
 // 该消息是用boost shared_ptr智能指针传递的，这意味着你可以根据需要存储它
 //  即不用担心它在下面被删除，又不必复制底层（underlying）数据。 
  void MessageCallBack(const nav_msgs::OccupancyGridPtr &cost_map);

  // 初始化地图
  void InitMap(double resultion,int x_size,int y_size);

  Node*** GetMap();
  
  // 进行Astar搜索
  void AstarPathFinding(Node* start_node,Node* target_node);

  std::vector<Eigen::Vector2d> GetPath();

  Eigen::Vector2d IndexToCoord(Eigen::Vector2i index);

  Eigen::Vector2i CoordToIndex(Eigen::Vector2d coord);

void SetObs(Eigen::Vector2i index);

void GetNeighborNode(Node* curr_node,std :: vector<Node*> &neighbor_set,
                                                 std :: vector<double> &neighbor_edge_set);

double GetHeu(Node* curr_node,Node* target_node);

bool IsFree(Node* node);

bool IsInMap(Eigen::Vector2i index);

bool has_map_ = false;

bool search_success_ = false;

// 知道为啥用一个容器来吗 
std:: vector<nav_msgs::OccupancyGridPtr>   cost_map_ ;
nav_msgs::OccupancyGridPtr   cost_map__ = nullptr ;


};