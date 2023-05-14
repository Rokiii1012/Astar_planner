#include "astar_planner.h"


void AstarPlanner :: MessageCallBack(const nav_msgs::OccupancyGridPtr &cost_map) {

    cost_map_ .push_back(cost_map);
    // cost_map__ = cost_map;
    ROS_WARN("recive_map_resolution:%d",cost_map_.front()->info.resolution);
    ROS_WARN("recive_map_width:%d",cost_map_.front()->info.width);
    ROS_WARN("recive_map_height:%d",cost_map_.front()->info.height);


  }



Eigen::Vector2d AstarPlanner::IndexToCoord(Eigen::Vector2i index) {
    Eigen::Vector2d coord;
    coord(0) = ((double)index(0) * 2.0 + 1.0) * resolution_ / 2.0;
    coord(1) = ((double)index(1) * 2.0 + 1.0) * resolution_ / 2.0;


    return coord;
}

Eigen::Vector2i AstarPlanner::CoordToIndex(Eigen::Vector2d coord) {
    
    Eigen::Vector2i index;
    index(0) = (int)(coord(0) / resolution_);
    index(1) = (int)(coord(1) / resolution_);

    return index;

}

void AstarPlanner::InitMap(double resultion,int x_size,int y_size) {

    // tag=new uint8_t[xSize*ySize];
    x_size_ = x_size;
    y_size_ = y_size;
    resolution_ = resultion;

    
    Map2d_ = new Node ** [x_size_];

    for (int i = 0; i < x_size_; i++) {

        Map2d_[i] = new Node * [y_size_];

        for (int j = 0;j < y_size_;j++) {   
            Eigen::Vector2i tmpIndex(i,j);

            Eigen::Vector2d tmpCoord = IndexToCoord(tmpIndex);
            // 在堆空间上创建节点new 类型（初始化），调用构造函数
            Map2d_[i][j] = new Node(tmpCoord,tmpIndex);
            
        }


    }
    has_map_ = true;
}

Node*** AstarPlanner::GetMap() {


    return Map2d_;
}



double AstarPlanner::GetHeu(Node* curr_node,Node* target_node) {

    double dx = curr_node->GetCoord()(0) - target_node->GetCoord()(0);
    double dy = curr_node->GetCoord()(1) - target_node->GetCoord()(1);
    double Heu = pow(dx * dx + dy *dy,0.5);

    return Heu;
}

void AstarPlanner::SetObs(Eigen::Vector2i index) {
    
    Map2d_[index(0)][index(1)]->SetTag();

}

bool AstarPlanner::IsFree(Node* node) {

    if (node->GetIndex()(0) <= x_size_ && node->GetIndex()(0) >= 0
         && node->GetIndex()(1) <= y_size_ && node->GetIndex()(0) >= 0
         && !node->GetTag()) {

      return true;

    }


    return false; 
}

bool AstarPlanner::IsInMap(Eigen::Vector2i index) {

    if(index(0) < x_size_ && index(0) >=0 
        && index(1) < y_size_ && index(1) >= 0) {

        return true;

    }

    return false;
}

void AstarPlanner::GetNeighborNode(Node* curr_node,std :: vector<Node*> &neighbor_set,
                                                 std :: vector<double> &neighbor_edge_set) {
  
 

  for (int k = -1; k < 2; k++) {

    for(int g = -1; g < 2; g++) {
        
        int i = curr_node->GetIndex()(0)+k;
        int j = curr_node->GetIndex()(1)+g;
        Eigen::Vector2i tmp_index;
        tmp_index(0) = i;
        tmp_index(1) = j;

        if (IsInMap(tmp_index)) {

            // 此处判断为什么是不等于-1，而不是等于0，因为我们需要访问新节点和更新容器中的节点
            if(!Map2d_[i][j]->GetTag() 
                && Map2d_[i][j]->GetOpenSetId() != -1) {
                    
                double dx = pow((Map2d_[i][j]->GetCoord()(0) - curr_node->GetCoord()(0)) ,2.0);
                double dy = pow((Map2d_[i][j]->GetCoord()(1) - curr_node->GetCoord()(1)),2.0);
                double edge_cost = pow(dx + dy,0.5);

                neighbor_edge_set.push_back(edge_cost);

                neighbor_set.push_back(Map2d_[i][j]);
        

            }

        }

    }

  }



}



void AstarPlanner::AstarPathFinding(Node* start,Node* target){
    
    // 初始化
    start_node_ = start;
    Node* current_node = nullptr;
    Node* neighbor_node = nullptr;
    // 将起点押入排序容器openset
    start->SetG(0.0);
    start->SetF(GetHeu(start,target));
    start->PushOpenSet();
    open_set_.insert(std::make_pair(start->GetF(),start));

    std :: vector<Node*> neighbor_node_set;
    std :: vector<double> neighbor_edge_set;
    ROS_WARN("INIT");



    while (!open_set_.empty()) {
        ROS_WARN("ENTER CIRCLE");
        // 将openset中的最优先的节点弹出
        current_node = open_set_.begin()->second;
        current_node->PopOpenSet();
        open_set_.erase(open_set_.begin());
        // ROS_WARN("CURRENT_NODE_XCOORD:%2f",current_node->GetCoord()(0));
        // ROS_WARN("CURRENT_NODE_XCOORD:%2f",current_node->GetCoord()(1));

        // ROS_WARN("CURRENT_NODE_XINDEX:%d",current_node->GetIndex()(0));
        // ROS_WARN("CURRENT_NODE_XINDEX:%d",current_node->GetIndex()(1));


        // 如果到达终点退出函数
        if (current_node->GetIndex() == target->GetIndex()) {
            

            target_node_=current_node;
            search_success_ = true;
            ROS_WARN("x:%2f",target->GetCoord()(0));
            ROS_WARN("x:%2f",target->GetCoord()(1));

            ROS_WARN("x:%d",target->GetIndex()(0));
            ROS_WARN("x:%d",target->GetIndex()(1));

            ROS_WARN("Search_success");

            return;
        }

        // 获取相邻的节点 
        GetNeighborNode(current_node,neighbor_node_set,neighbor_edge_set);

        for (int i = 0;i < neighbor_node_set.size();i++) {
            // 如果相邻节点是新节点,计算g和f值直接加入容器
            if(neighbor_node_set[i]->GetOpenSetId() == 0) {

                neighbor_node_set[i]->SetG(neighbor_edge_set[i]);
                neighbor_node_set[i]->SetF(neighbor_node_set[i]->GetG() + GetHeu(current_node,target));
                neighbor_node_set[i]->SetParentNode(current_node);
                neighbor_node_set[i]->PushOpenSet();

                open_set_.insert(std::make_pair(neighbor_node_set[i]->GetF(),neighbor_node_set[i]));
                
            } else if (neighbor_node_set[i]->GetOpenSetId() == 1) {

                if(neighbor_node_set[i]->GetG() > current_node->GetG() + neighbor_edge_set[i]) {

                    neighbor_node_set[i]->SetG(current_node->GetG() + neighbor_edge_set[i]);
                    neighbor_node_set[i]->SetF(neighbor_node_set[i]->GetG() + GetHeu(current_node,target));
                    neighbor_node_set[i]->SetParentNode(current_node);


                } 

            } else {


                continue;
            }

        }

    }
    

    
}

std::vector<Eigen::Vector2d> AstarPlanner::GetPath() {

    std::vector<Node*> path;
    std::vector<Eigen::Vector2d> path_coord;
    Node* tmp = target_node_; 
    while  (start_node_->GetIndex() != tmp->GetIndex()) {


        path.push_back(tmp);
        path_coord.push_back(tmp->GetCoord());
        tmp = tmp->GetParentNode();

    }

    path.push_back(start_node_);
    path_coord.push_back(start_node_->GetCoord());
    
    reverse(path.begin(),path.end());
    reverse(path_coord.begin(),path_coord.end());

    return path_coord;


}
