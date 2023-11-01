#include "planner_vis.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <algorithm>
#include <execution>

// #include <ros/ros.h>
// #include <visualization_msgs/Marker.h>
// #include <Eigen/Dense>
// #include <vector>

using namespace OAC;
using namespace Eigen;
using namespace ros;
using namespace std;

namespace OAC
{
namespace visualization
{
/**
 * @brief make frame to show pose and shape
 */
vector<Vector4d> generateFrame(const vector<Vector3d>& pts, const vector<float>& color_pts,
                               const vector<Quaterniond>& orientations);

void visWorld(World* world, Publisher* world_vis_pub, Publisher* grid_vis_pub)
{
  if (world_vis_pub == NULL || !world->has_map_)
    return;
  
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  for(auto& grid:world->effect_grid_)
  {
    Vector3d coor_round = world->index2coord(grid);
    pcl::PointXYZ pt_add;
    pt_add.x = coor_round(0);
    pt_add.y = coor_round(1);
    pt_add.z = coor_round(2);
    cloud_vis.points.push_back(pt_add);
  }

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;

  world->subMap_.setTimestamp(ros::Time::now().toNSec());
  // world->gridMap_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap gridMapMsg;
  grid_map::GridMapRosConverter::toMessage(world->subMap_, gridMapMsg);
  // grid_map::GridMapRosConverter::toMessage(world->gridMap_, gridMapMsg);

  grid_vis_pub->publish(gridMapMsg);

  sensor_msgs::PointCloud2 map_vis;
  pcl::toROSMsg(cloud_vis, map_vis);

  map_vis.header.frame_id = "world";
  world_vis_pub->publish(map_vis);
}

void visSurf(const vector<Node*>& solution, Publisher* surf_vis_pub)
{
  if (surf_vis_pub == NULL)
    return;
  pcl::PointCloud<pcl::PointXYZRGB> surf_point;
  pcl::PointXYZRGB pt;
  for (const auto& node : solution)
  {
    for (const auto& point : node->plane_->plane_pts)
    {
      pt.x = point(0);
      pt.y = point(1);
      pt.z = point(2);
      pt.r = pt.g = pt.b = 0;
      pt.a = 1.0f;

      surf_point.push_back(pt);
    }
  }

  surf_point.width = surf_point.points.size();
  surf_point.height = 1;
  surf_point.is_dense = true;

  sensor_msgs::PointCloud2 map_vis;
  pcl::toROSMsg(surf_point, map_vis);

  map_vis.header.frame_id = "world";
  surf_vis_pub->publish(map_vis);
}

void visOriginAndGoal(const vector<Node*>& nodes, Publisher* origin_and_goal_vis_pub)
{
  if (origin_and_goal_vis_pub == NULL)
    return;
  visualization_msgs::Marker Sphere;
  Sphere.header.frame_id = "world";
  Sphere.header.stamp = ros::Time::now();
  Sphere.ns = "Goal";
  if (nodes.empty())
  {
    Sphere.action = visualization_msgs::Marker::DELETE;
    origin_and_goal_vis_pub->publish(Sphere);
    return;
  }
  Sphere.action = visualization_msgs::Marker::ADD;
  Sphere.pose.orientation.w = 1.0;
  Sphere.id = 0;
  Sphere.type = visualization_msgs::Marker::SPHERE_LIST;

  Sphere.scale.x = Sphere.scale.y = Sphere.scale.z = 0.2f;

  Sphere.color.g = Sphere.color.b = Sphere.color.r = Sphere.color.a = 1.0f;

  geometry_msgs::Point pt;
  for (const auto& node : nodes)
  {
    Vector3d coord = node->position_;
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = coord(2);

    Sphere.points.push_back(pt);
  }
  origin_and_goal_vis_pub->publish(Sphere);
}

void visPath(const vector<Node*>& solution, Publisher* path_vis_pub, const Eigen::Vector3d &start_pt)
{
  if (path_vis_pub == NULL)
    return;
  visualization_msgs::Marker Points, Line, Frame;
  Frame.header.frame_id = Points.header.frame_id = Line.header.frame_id = "world";
  Frame.header.stamp = Points.header.stamp = Line.header.stamp = ros::Time::now();
  Line.pose.orientation.w = Frame.pose.orientation.w = 1.0f;
  Frame.ns = Points.ns = Line.ns = "Path";
  Points.id = 0;
  Line.id = 1;
  Frame.id = 2;

  Points.type = visualization_msgs::Marker::POINTS;
  Line.type = visualization_msgs::Marker::LINE_STRIP;
  Frame.type = visualization_msgs::Marker::LINE_LIST;

  Frame.scale.x = 0.05;
  Points.scale.x = 0.1;
  Points.scale.y = 0.1;
  Line.scale.x = 0.1;

  Points.color.g = Points.color.a = 1.0f;
  Line.color.b = Line.color.a = 1.0f;
  Frame.color.g = Frame.color.b = Frame.color.a = 1.0f;
  std::ofstream output;
  output.open("/home/parallels/1.txt", std::ios::app);

  if (solution.size() > 1)
  {
    vector<Vector3d> pts;
    vector<float> pts_tra;
    vector<Quaterniond> orientations;
    for (const auto& node : solution)
    {
      pts.push_back(node->position_);
      pts_tra.push_back(node->plane_->traversability);
    }

    geometry_msgs::Point pt;
    for (const auto& coord : pts)
    {
      pt.x = coord(0);
      pt.y = coord(1);
      pt.z = coord(2);

      Points.points.push_back(pt);
      Line.points.push_back(pt);
    }

    for (size_t i = 0; i < solution.size() - 1; i++)
    {
      Vector3d q = solution[i + 1]->position_ - solution[i]->position_;
      Vector3d e_z = solution[i]->plane_->normal_vector;

      Vector3d e_x = q - (q.dot(e_z)) * q;
      e_x.normalize();

      Vector3d e_y = e_z.cross(e_x);

      Matrix3d R;
      R << e_x(0), e_y(0), e_z(0), e_x(1), e_y(1), e_z(1), e_x(2), e_y(2), e_z(2);
      Quaterniond quaternion(R);
      orientations.push_back(quaternion);
    }

    // make frame
    vector<Vector4d> frame_list = generateFrame(pts, pts_tra, orientations);
    for (const auto& coord : frame_list)
    {
      pt.x = coord(0);
      pt.y = coord(1);
      pt.z = coord(2);
      std_msgs::ColorRGBA color;
      color.r = 1;
      color.g = coord(3);
      color.b = 0.05;
      color.a = 0.8;

      Frame.points.push_back(pt);
      Frame.colors.push_back(color);
    }
  }
  else
  {
    Frame.action = Points.action = Line.action = visualization_msgs::Marker::DELETE;
  }

  path_vis_pub->publish(Points);
  path_vis_pub->publish(Line);
  path_vis_pub->publish(Frame);
}

void visTree(const vector<Node*>& tree, Publisher* tree_vis_pub)
{
  if (tree_vis_pub == NULL)
    return;
  visualization_msgs::Marker Points, Line;
  Points.header.frame_id = Line.header.frame_id = "world";
  Points.header.stamp = Line.header.stamp = ros::Time::now();
  Points.ns = Line.ns = "Tree";
  Points.action = Line.action = visualization_msgs::Marker::ADD;
  Points.pose.orientation.w = Line.pose.orientation.w = 1.0f;
  Points.id = 0;
  Line.id = 1;

  Points.type = visualization_msgs::Marker::POINTS;
  Line.type = visualization_msgs::Marker::LINE_LIST;

  Points.scale.x = Points.scale.y = 0.05;
  Line.scale.x = 0.01;

  // Points are green and Line Strip is blue
  Points.color.g = Points.color.a = 0.5f;
  // Points.color.g = Points.color.r = 255*traversability;
  Line.color.b = Line.color.a = 0.75f;

  geometry_msgs::Point pt;
  geometry_msgs::Point parent_pt;
  for (const auto& node : tree)
  {
    pt.x = node->position_(0);
    pt.y = node->position_(1);
    pt.z = node->position_(2);
    // std_msgs::ColorRGBA color;
    // color.r=color.g=node->plane_->traversability;
    // color.b=0;
    // color.a=1;

    // Points.colors.push_back(color);
    Points.points.push_back(pt);

    if (node->parent_ != NULL)  // skip the root node
    {
      Line.points.push_back(pt);
      parent_pt.x = node->parent_->position_(0);
      parent_pt.y = node->parent_->position_(1);
      parent_pt.z = node->parent_->position_(2);
      Line.points.push_back(parent_pt);
    }
  }
  tree_vis_pub->publish(Points);
  tree_vis_pub->publish(Line);
}

vector<Vector4d> generateFrame(const vector<Vector3d>& pts, const vector<float>& color_pts,
                               const vector<Quaterniond>& orientations)
{
  float frame_w = 0.75;
  float frame_h = 0.36;

  vector<Vector4d> Frame_list;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  geometry_msgs::Point p3;
  geometry_msgs::Point p4;

  for (size_t i = 1; i < pts.size() - 1; i++)
  {
    Vector3d p1_tmp, p2_tmp, p3_tmp, p4_tmp;
    Vector3d p1_, p2_, p3_, p4_;
    Vector4d _p1, _p2, _p3, _p4;
    p1_tmp(0) = 0;
    p1_tmp(1) = +frame_w * 0.5;
    p1_tmp(2) = +frame_h * 0.5;
    p2_tmp(0) = 0;
    p2_tmp(1) = -frame_w * 0.5;
    p2_tmp(2) = +frame_h * 0.5;
    p3_tmp(0) = 0;
    p3_tmp(1) = +frame_w * 0.5;
    p3_tmp(2) = -frame_h * 0.5;
    p4_tmp(0) = 0;
    p4_tmp(1) = -frame_w * 0.5;
    p4_tmp(2) = -frame_h * 0.5;

    Matrix3d ratation_matrix = orientations[i].matrix();

    p1_ = ratation_matrix * p1_tmp + pts[i];
    p2_ = ratation_matrix * p2_tmp + pts[i];
    p3_ = ratation_matrix * p3_tmp + pts[i];
    p4_ = ratation_matrix * p4_tmp + pts[i];

    _p1 << p1_(0), p1_(1), p1_(2), color_pts[i];
    _p2 << p2_(0), p2_(1), p2_(2), color_pts[i];
    _p3 << p3_(0), p3_(1), p3_(2), color_pts[i];
    _p4 << p4_(0), p4_(1), p4_(2), color_pts[i];

    Frame_list.push_back(_p1);
    Frame_list.push_back(_p2);
    Frame_list.push_back(_p2);
    Frame_list.push_back(_p4);
    Frame_list.push_back(_p4);
    Frame_list.push_back(_p3);
    Frame_list.push_back(_p3);
    Frame_list.push_back(_p1);
  }
  return Frame_list;
}

void visTrajectory(const std::vector<Eigen::Vector3d> &waypoints, 
                   const Eigen::MatrixX3d &coefficientMatrix,
                   const std::vector<double> &timeVector,
                   ros::Publisher* traj_vis_pub){

    // // 构造Marker消息
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "camera_init";
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "trajectory";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.scale.x = 0.1;  // 线宽
    // marker.color.r = 1.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.color.a = 1.0;

    // 构造Path消息
    nav_msgs::Path path;
    path.header.frame_id = "camera_init";
    path.header.stamp = ros::Time::now();

    // 遍历每个曲线段，生成Marker和Path消息
    for (size_t i = 0; i < coefficientMatrix.rows(); i += 6)
    {
        // 获取当前曲线段的系数
        Eigen::Vector3d a = coefficientMatrix.row(i + 0);
        Eigen::Vector3d b = coefficientMatrix.row(i + 1);
        Eigen::Vector3d c = coefficientMatrix.row(i + 2);
        Eigen::Vector3d d = coefficientMatrix.row(i + 3);
        Eigen::Vector3d e = coefficientMatrix.row(i + 4);
        Eigen::Vector3d f = coefficientMatrix.row(i + 5);

        // 生成当前曲线段的轨迹点
        std::vector<Eigen::Vector3d> segmentPoints;
        for (double t = 0.0; t <= timeVector[i]; t += 0.001)
        {
            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t3 * t;
            double t5 = t4 * t;

            Eigen::Vector3d point = a + b * t + c * t2 + d * t3 + e * t4 + f * t5;
            segmentPoints.push_back(point);
        }

        // // 将当前曲线段的轨迹点添加到Marker消息中
        // for (const auto& point : segmentPoints)
        // {
        //     geometry_msgs::Point markerPoint;
        //     markerPoint.x = point.x();
        //     markerPoint.y = point.y();
        //     markerPoint.z = point.z();
        //     marker.points.push_back(markerPoint);
        // }

        // 将当前曲线段的轨迹点添加到Path消息中
        for (const auto& point : segmentPoints)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            path.poses.push_back(pose);
        }
    }

    // 发布Marker和Path消息
    // markerPub.publish(marker);
    traj_vis_pub->publish(path);
}

}  // namespace visualization
}  // namespace OAC
