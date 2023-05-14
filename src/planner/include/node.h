#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#define inf 1<<18
using namespace Eigen;



class  Node{
 public:
    explicit Node(Vector2d coord,Vector2i index)
            : coord_(coord),index_(index),parent_(nullptr),
            gScore_(inf),fScore_(inf),tag_(0) ,in_open_set_(0) {}
            
    // ~Node() = default;

    Vector2d GetCoord() {
        return coord_;
    }

    Vector2i  GetIndex() {
        return index_;
    }

    void SetParentNode(Node* parent_node) {
        parent_ = parent_node;
    }
    
    Node* GetParentNode() {
        return parent_;
    }

    double GetG() {    
        return gScore_;
    }

    double GetF() {
        return fScore_;
    }

    void SetG(double g) {

        gScore_=g;
    }

    void SetF(double f) {

        fScore_=f;
    }

    void SetTag() {
        tag_ = 1;
    }

    int GetTag() {

        return tag_;
    }

    void PushOpenSet() {

        in_open_set_ = 1;
    }

    void PopOpenSet() {

        in_open_set_ = -1;
    }

    int GetOpenSetId() {

        return in_open_set_;
    }


 private:
    // tag为0则无障碍物，为1则有障碍物
    int tag_;
    // 1为加入openset，0为未加入,加入过为-1，也就是访问过
    int in_open_set_;
    Eigen::Vector2d coord_;
    Eigen::Vector2i index_;
    Node* parent_;
    double gScore_,fScore_;





};