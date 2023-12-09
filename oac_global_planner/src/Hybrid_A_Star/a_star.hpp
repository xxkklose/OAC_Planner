#pragma once

#include <cmath>
#include <ros/ros.h>
#include <queue>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "tic_toc.h"
#include <fstream>

using namespace std;
using namespace Eigen;

std::ofstream outputFile;

double AStar_resolution;

const int direction[8][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, 
                       {1, 1}, {-1, -1}, {-1, 1}, {1, -1}};

//函数对象类
template <typename T>
class cmp
{
public:
    //重载 () 运算符
    bool operator()(T a, T b)
    {
        return a->f > b->f;
    }
};

struct TreeNode
{
    double x;
    double y;
    double f;
    double g;
    double h;
    TreeNode* parent;
    bool isVisited;
    int x_index;
    int y_index;
    bool operator==(const TreeNode& node){
        return (Vector2d(x, y) - Vector2d(node.x, node.y)).norm() < 0.01;
    }
    void updateH(TreeNode end){
       //欧式距离
       h = sqrt(pow(x - end.x, 2) + pow(y - end.y, 2));
    }
    void updateF()
    {
        f = g + h;
    }
    TreeNode(){
        x = 0;
        y = 0;
        f = 0;
        g = INFINITY;
        h = 0;
        parent = NULL;
        isVisited = false;
    }
};


class AStar
{    

public:
    AStar(){
        start_node_ = new TreeNode();
        goal_node_ = new TreeNode();
    }

    ~AStar() {
        // ROS_INFO("AStar delete begin");
        // if(start_node_ != nullptr)
        // {
        //     try{
        //         delete start_node_;
        //     }catch(exception e){
        //         ROS_ERROR("delete start_node_ error");
        //     }
        // }
        // if(goal_node_ != nullptr)
        // {
        //     try{
        //         delete goal_node_;
        //     }catch(exception e){
        //         ROS_ERROR("delete start_node_ error");
        //     }
        // }

        // for(int i = 0; i < 250; i++){
        //     for(int j = 0; j < 250; j++){
        //         if(node_map[i][j] != nullptr)
        //         {
        //             try{
        //                 delete node_map[i][j];
        //                 node_map[i][j] = nullptr;
        //             }catch(exception e){
        //                 ROS_ERROR("delete start_node_ error");
        //             }
        //         }
        //     }
        // }
        // ROS_INFO("AStar delete");
    }


    bool initParam(const grid_map::GridMap& map, double resolution, Vector3d start, Vector3d goal){
        map_ = map;
        AStar_resolution = resolution;
        if(!map_.exists("elevation")){
            ROS_ERROR("Map does not have elevation layer!");
            return false;
        }else if(map_.isInside(grid_map::Position(start(0), start(1))) == false){
            ROS_ERROR("Start point is not in the map!");
            return false;
        }else if(map_.isInside(grid_map::Position(goal(0), goal(1))) == false){
            ROS_ERROR("Goal point is not in the map!");
            return false;
        }else if(map_.atPosition("dilatation_barrier", grid_map::Position(goal(0), goal(1))) > 0.5){
            ROS_ERROR("Goal point is in the obstacle!");
            return false;
        }else{
            grid_map::Index start_index;
            grid_map::Position start_position;
            map_.getIndex(grid_map::Position(start(0), start(1)), start_index);
            map_.getPosition(start_index, start_position);
            start_node_->x = start_position.x();
            start_node_->y = start_position.y();
            grid_map::Index goal_index;
            grid_map::Position goal_position;
            map_.getIndex(grid_map::Position(goal(0), goal(1)), goal_index);
            map_.getPosition(goal_index, goal_position);
            goal_node_->x = goal_position.x();
            goal_node_->y = goal_position.y();
            start_node_->updateH(*goal_node_);
            start_node_->g = 0;
            start_node_->updateF();
            start_node_->isVisited = true;
            start_node_->x_index = 124;
            start_node_->y_index = 124;
            open_list_.insert(make_pair(start_node_->f, start_node_));
            // for(int i = 0; i < 250; i++){
            //     for(int j = 0; j < 250; j++){
            //         node_map[i][j] = new TreeNode();
            //         node_map[i][j]->x = start_position.x() + (i - 124) * AStar_resolution;
            //         node_map[i][j]->y = start_position.y() - (j - 124) * AStar_resolution;
            //         node_map[i][j]->x_index = i;
            //         node_map[i][j]->y_index = j;
            //     }
            // }
            for(int i = 0; i < 250; i++){
                vector<TreeNode*> temp_row;
                for(int j = 0; j < 250; j++){
                    TreeNode* temp = new TreeNode();
                    temp->x = start_position.x() + (i - 124) * AStar_resolution;
                    temp->y = start_position.y() - (j - 124) * AStar_resolution;
                    temp->x_index = i;
                    temp->y_index = j;
                    temp_row.push_back(temp);
                }
                node_map.push_back(temp_row);
            }


            int start_index_x = (start_node_->x - start_node_->x) / AStar_resolution;
            int start_index_y = (start_node_->y - start_node_->y) / AStar_resolution;
            // TreeNode* start_node = node_map[start_index_x + 124][start_index_y + 124];
            TreeNode* start_node = node_map[124][124];
        }
        return true;
    }

    void process(){
        TicToc t;

        TreeNode* current_node = nullptr;
        int curr_index_x = 124;
        int curr_index_y = 124;

        while(!open_list_.empty() && t.toc() < 1e4){
            current_node = open_list_.begin()->second;
            current_node->isVisited = true;
            curr_index_x = current_node->x_index;
            curr_index_y = current_node->y_index;
            open_list_.erase(open_list_.begin());
            // 判断是否到达终点
            if(*current_node == *goal_node_){
                goal_node_->parent = current_node->parent;
                return;
            }

            // 遍历八个方向
            for(int i = 0; i < 8; i++){
                int x_index = min(max(curr_index_x + direction[i][0], 0), 249);
                int y_index = min(max(curr_index_y + direction[i][1], 0), 249);
                TreeNode* neighbor_node = node_map[x_index][y_index];
                if(isFree(*neighbor_node) == false){
                    continue;
                }
                if(neighbor_node->isVisited == true){
                    continue;
                }

                double Cnm = sqrt(abs(direction[i][0])  + abs(direction[i][1]));
                if(neighbor_node->g == INFINITY)
                {
                    neighbor_node->g = current_node->g + Cnm;
                    neighbor_node->updateH(*goal_node_);
                    neighbor_node->updateF();
                    neighbor_node->parent = current_node;
                    open_list_.insert(make_pair(neighbor_node->f, neighbor_node));
                }
                else if(neighbor_node->g > current_node->g + Cnm)
                {
                    neighbor_node->g = current_node->g + Cnm;
                    neighbor_node->updateH(*goal_node_);
                    neighbor_node->updateF();
                    neighbor_node->parent = current_node;
                }
            }


        }
    }

    vector<Vector3d> returnPath(){
        vector<Vector3d> path;
        TreeNode* current_node = goal_node_;
        while(current_node != NULL){
            grid_map::Position position(current_node->x, current_node->y);
            if(map_.isInside(position) == false){
                continue;
            }
            path.push_back(Vector3d(current_node->x, current_node->y, map_.atPosition("elevation", position) + 0.68));
            current_node = current_node->parent;
        }
        outputFile.close();
        return path;
    }

    bool isFree(TreeNode node){
        grid_map::Position position(node.x, node.y);
        if(map_.isInside(position) == false){
            return false;
        }
        if(map_.atPosition("dilatation_barrier", position) > 0.5){
            return false;
        }
        return true;
    }

private:
    grid_map::GridMap map_;

    std::multimap<double, TreeNode*> open_list_;
    // TreeNode* node_map[250][250];
    vector<vector<TreeNode*>> node_map;


    TreeNode* start_node_ = nullptr;
    TreeNode* goal_node_ = nullptr;
};

