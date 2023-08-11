#include "backward.hpp"
#include "planner.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace OAC;
using namespace OAC::visualization;
using namespace OAC::planner;

namespace backward
{
backward::SignalHandling sh;
}
std::ofstream outputFile;

// ros related
ros::Subscriber map_sub, wp_sub;

ros::Publisher grid_map_vis_pub;
ros::Publisher path_vis_pub;
ros::Publisher goal_vis_pub;
ros::Publisher surf_vis_pub;
ros::Publisher tree_vis_pub;
ros::Publisher path_interpolation_pub;
ros::Publisher tree_tra_pub;
ros::Publisher pose_pub_to_control;
ros::Publisher traj_jerk_vis_pub;

// indicate whether the robot has a moving goal
bool has_goal = false;

// simulation param from launch file
double resolution;
double goal_thre;
double step_size;
double h_surf_car;
double max_initial_time;
double radius_fit_plane;
FitPlaneArg fit_plane_arg;
double neighbor_radius;

// useful global variables
Vector3d start_pt;
Vector3d target_pt;
World* world = NULL;
Minimum_jerk mj = Minimum_jerk();
PFRRTStar* pf_rrt_star = NULL;

// function declaration
void rcvWaypointsCallback(const nav_msgs::Path& wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map);
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub);
void findSolution();
void callPlanner();

/**
 *@brief receive goal from rviz
 */
void rcvWaypointsCallback(const nav_msgs::Path& wp)
{
  if (!world->has_map_)
    return;
  has_goal = true;
  target_pt = Vector3d(wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z);
  ROS_INFO("Receive the planning target");
}

/**
 *@brief receive point cloud to build the grid map
 */
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(pointcloud_map, cloud);

  world->initGridMap(cloud);

  for (const auto& pt : cloud)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    // if(grid_map_count_[idx(0)][idx(1)][idx(2)] >= 1){
    //     grid_map_[idx(0)][idx(1)][idx(2)]=false;
    // }
    world->setObs(obstacle);
  }
  for (const auto& pt : cloud)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    world->addObs(obstacle);
  }
  visWorld(world, &grid_map_vis_pub);
}

/**
 *@brief Linearly interpolate the generated path to meet the needs of local planning
 */
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* path_interpolation_pub)
{
  if (path_interpolation_pub == NULL)
    return;
  Float32MultiArray msg;
  for (size_t i = 0; i < solution.size(); i++)
  {
    if (i == solution.size() - 1)
    {
      msg.data.push_back(solution[i]->position_(0));
      msg.data.push_back(solution[i]->position_(1));
      msg.data.push_back(solution[i]->position_(2));
    }
    else
    {
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
      Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;
      for (size_t j = 0; j < interpolation_num; j++)
      {
        Vector3d interpt = solution[i]->position_ + diff_pt * (float)j / interpolation_num;
        msg.data.push_back(interpt(0));
        msg.data.push_back(interpt(1));
        msg.data.push_back(interpt(2));
      }
    }
  }
  path_interpolation_pub->publish(msg);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void findSolution()
{
  printf("=========================================================================\n");
  ROS_INFO("Start calling PF-RRT*");
  Path solution = Path();

  pf_rrt_star->initWithGoal(start_pt, target_pt);
  // pf_rrt_star->initWithGoal(target_pt, start_pt);

  // Case1: The PF-RRT* can't work at when the origin can't be project to surface
  if (pf_rrt_star->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }
  // Case2: If both the origin and the target can be projected,the PF-RRT* will execute
  //       global planning and try to generate a path
  else if (pf_rrt_star->state() == Global)
  {
    ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;
    double max_time = 100.0;

    while (solution.type_ == Path::Empty && max_time < max_initial_time)
    {
      solution = pf_rrt_star->planner(max_iter, max_time);
      max_time += 100.0;
    }
    mj.waypoints.clear();
    for(const auto &node : solution.nodes_){
      mj.waypoints.push_back(node->position_);
    }
    // ROS_WARN("1");
    mj.waypoints.push_back(start_pt);
    Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6*(mj.waypoints.size()-1), 3);
    // mj.getTimeVector(mj.waypoints,0.4,0.2); //TODO:max_vel, max_acc
    // mj.solve_minimum_jerk(mj.waypoints, mj.start_vel, mj.start_acc, coefficientMatrix);
    // // mj.solve_minimum_jerk(mj.waypoints, {}, {}, coefficientMatrix); //暂时先用零向量代替


    // visTrajectory(mj.waypoints, coefficientMatrix, mj.timeVector, mj.traj_jerk_vis_pub_);

    if (!solution.nodes_.empty())
      ROS_INFO("Get a global path!");
    else
      ROS_WARN("No solution found!");
  }
  // Case3: If the origin can be projected while the target can not,the PF-RRT*
  //       will try to find a temporary target for transitions.
  else
  {
    ROS_INFO("Starting PF-RRT* algorithm at the state of rolling planning");
    int max_iter = 1500;
    double max_time = 100.0;

    solution = pf_rrt_star->planner(max_iter, max_time);
    mj.waypoints.clear();
    for(const auto &node : solution.nodes_){
      mj.waypoints.push_back(node->position_);
    }

    mj.waypoints.push_back(start_pt);
    Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6*(mj.waypoints.size()-1), 3);
    // mj.getTimeVector(mj.waypoints,0.4,0.2); //TODO:max_vel, max_acc
    // mj.solve_minimum_jerk(mj.waypoints, mj.start_vel, mj.start_acc, coefficientMatrix);
    // // mj.solve_minimum_jerk(mj.waypoints, {}, {}, coefficientMatrix);

    // visTrajectory(mj.waypoints, coefficientMatrix, mj.timeVector, mj.traj_jerk_vis_pub_);

    if (!solution.nodes_.empty())
      ROS_INFO("Get a sub path!");
    else
      ROS_WARN("No solution found!");
  }
  ROS_INFO("End calling PF-RRT*");
  printf("=========================================================================\n");

  pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution.nodes_, &path_vis_pub, start_pt);
  visSurf(solution.nodes_, &surf_vis_pub);

  // When the PF-RRT* generates a short enough global path,it's considered that the robot has
  // reached the goal region.
  if (solution.type_ == Path::Global && EuclideanDistance(pf_rrt_star->origin(), pf_rrt_star->target()) < goal_thre)
  {
    has_goal = false;
    visOriginAndGoal({}, &goal_vis_pub);  // Passing an empty set to delete the previous display
    visPath({}, &path_vis_pub, start_pt);
    ROS_INFO("The Robot has achieved the goal!!!");
  }

  if (solution.type_ == Path::Empty)
    visPath({}, &path_vis_pub, start_pt);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void callPlanner()
{
  static double init_time_cost = 0.0;
  if (!world->has_map_)
    return;

  // The tree will expand at a certain frequency to explore the space more fully
  if (!has_goal && init_time_cost < 1000)
  {
    timeval start;
    gettimeofday(&start, NULL);
    pf_rrt_star->initWithoutGoal(start_pt);
    timeval end;
    gettimeofday(&end, NULL);
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    if (pf_rrt_star->state() == WithoutGoal)
    // if (pf_rrt_star->state() == WithoutGoal && (int)(pf_rrt_star->tree().size() < 3000))
    {
      int max_iter = 550;
      double max_time = 100.0;
      pf_rrt_star->planner(max_iter, max_time);
      ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star->tree().size()));
    }
    else if(pf_rrt_star->state() != WithoutGoal)
      ROS_WARN("The start point can't be projected,unable to execute PF-RRT* algorithm");
  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal)
  {
    findSolution();
    init_time_cost = 0.0;
  }
  // The expansion of tree will stop after the process of initialization takes more than 1s
  else
    ROS_INFO("The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh("~");

  map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);
  wp_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);

  grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
  path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 40);
  goal_vis_pub = nh.advertise<visualization_msgs::Marker>("goal_vis", 1);
  surf_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("surf_vis", 100);
  tree_vis_pub = nh.advertise<visualization_msgs::Marker>("tree_vis", 40);
  tree_tra_pub = nh.advertise<std_msgs::Float32MultiArray>("tree_tra", 40);
  path_interpolation_pub = nh.advertise<std_msgs::Float32MultiArray>("global_path", 1000);
  pose_pub_to_control = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 40);
  traj_jerk_vis_pub = nh.advertise<nav_msgs::Path>("trajectory_path", 40);

  nh.param("map/resolution", resolution, 0.1);

  nh.param("planning/goal_thre", goal_thre, 1.0);
  nh.param("planning/step_size", step_size, 0.2);
  nh.param("planning/h_surf_car", h_surf_car, 0.1);
  nh.param("planning/neighbor_radius", neighbor_radius, 1.0);

  nh.param("planning/w_fit_plane", fit_plane_arg.w_total_, 0.4);
  nh.param("planning/w_flatness", fit_plane_arg.w_flatness_, 4000.0);
  nh.param("planning/w_slope", fit_plane_arg.w_slope_, 0.4);
  nh.param("planning/w_sparsity", fit_plane_arg.w_sparsity_, 0.4);
  nh.param("planning/ratio_min", fit_plane_arg.ratio_min_, 0.25);
  nh.param("planning/ratio_max", fit_plane_arg.ratio_max_, 0.4);
  nh.param("planning/conv_thre", fit_plane_arg.conv_thre_, 0.1152);

  nh.param("planning/radius_fit_plane", radius_fit_plane, 1.0);

  nh.param("planning/max_initial_time", max_initial_time, 1000.0);

  // // Initialization
  world = new World(resolution);
  
  pf_rrt_star = new PFRRTStar(h_surf_car, world);

  // Set argument of PF-RRT*
  pf_rrt_star->setGoalThre(goal_thre);
  pf_rrt_star->setStepSize(step_size);
  pf_rrt_star->setFitPlaneArg(fit_plane_arg);
  pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
  pf_rrt_star->setNeighborRadius(neighbor_radius);

  pf_rrt_star->goal_vis_pub_ = &goal_vis_pub;
  pf_rrt_star->tree_vis_pub_ = &tree_vis_pub;
  pf_rrt_star->tree_tra_pub_ = &tree_tra_pub;

  mj.traj_jerk_vis_pub_ = &traj_jerk_vis_pub;
  
  tf::TransformListener listener;

  while (ros::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);

    // Update the position of the origin
    tf::StampedTransform transform;

    while (true && ros::ok())
    {
      try
      {
        listener.lookupTransform("camera_init", "body", ros::Time(0), transform);  //查询变换
        break;
      }
      catch (tf::TransformException& ex)
      {
        continue;
      }
    }
    start_pt << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
   
    //set message robot_pose 
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "camera_init";
    // pose_msg.pose.position.x = transform.getOrigin().x() - 0.17193;
    pose_msg.pose.position.x = transform.getOrigin().x();
    pose_msg.pose.position.y = transform.getOrigin().y();
    // pose_msg.pose.position.z = transform.getOrigin().z() - 0.13942;
    pose_msg.pose.position.z = transform.getOrigin().z();
    pose_msg.pose.orientation.w = transform.getRotation().getW();
    pose_msg.pose.orientation.x = transform.getRotation().getX();
    pose_msg.pose.orientation.y = transform.getRotation().getY();
    pose_msg.pose.orientation.z = transform.getRotation().getZ();
    //save vel_direction to minimum_jerk
    Eigen::Vector3d unit_vector = {1.0, 0.0, 0.0};
    Eigen::Quaterniond tmp_quaternion(transform.getRotation().getW(),
                                      transform.getRotation().getX(),
                                      transform.getRotation().getY(),
                                      transform.getRotation().getZ());
    
    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d rotationMatrix = tmp_quaternion.toRotationMatrix();
    // 转换到camera_init坐标系下
    mj.start_vel = rotationMatrix * (0.01*unit_vector);
    mj.start_acc = rotationMatrix * (0.01*unit_vector);
    // mj.start_vel = rotationMatrix * unit_vector;
    // mj.start_acc = rotationMatrix * unit_vector;
    pose_pub_to_control.publish(pose_msg);

    // Execute the callback functions to update the grid map and check if there's a new goal
    ros::spinOnce();
    // Call the PF-RRT* to work
    callPlanner();
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 100);  // Cycle in 100ms
  }
  return 0;
}
