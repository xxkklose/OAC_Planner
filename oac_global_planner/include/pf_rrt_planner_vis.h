/**
 *  This file is used for visualization
 *  Author: jianzhuozhu
 *  Date: 2021-7-27
 */

#ifndef OAC_VIS_H
#define OAC_VIS_H

#include "pf_rrt_planner_classes.h"

namespace OAC
{
namespace visualization
{

/**
 * @brief visualize the grid map
 */
void visWorld(World* world, ros::Publisher* grid_map_vis_hub);

/**
 * @brief visualize the plane of the nodes along the path generated by PF-RRT*
 */
void visSurf(const std::vector<Node*> &solution,ros::Publisher* surf_vis_pub);

/**
 * @brief visualize the start point and the end point of PF-RRT*
 */
void visOriginAndGoal(const std::vector<Node*> &pts,ros::Publisher* origin_and_goal_vis_pub);

/**
 * @brief visualize the path generated by PF-RRT*
 */
void visPath(const std::vector<Node*> &solution,ros::Publisher* path_vis_pub, const Eigen::Vector3d &start_pt);

/**
 * @brief visualize the tree of PF-RRT*
 */
void visTree(const std::vector<Node*> &tree,ros::Publisher* tree_vis_pub);

void visTrajectory(const std::vector<Eigen::Vector3d> &waypoints, 
                   const Eigen::MatrixX3d &coefficientMatrix,
                   const std::vector<double> &timeVector,
                   ros::Publisher* traj_vis_pub);

}

}
#endif