#include "backward.hpp"
#include "planner.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <queue>
#include <fstream>
#include <thread>
#include <mutex>
#include <chrono>
#include <execution>  
#include <algorithm>  

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
ros::Subscriber map_sub, wp_sub, pose_sub;

ros::Publisher grid_map_vis_pub;
ros::Publisher path_vis_pub;
ros::Publisher goal_vis_pub;
ros::Publisher surf_vis_pub;
ros::Publisher tree_vis_pub;
ros::Publisher path_interpolation_pub;
ros::Publisher tree_tra_pub;
ros::Publisher pose_pub_to_control;
ros::Publisher traj_jerk_vis_pub;
ros::Publisher pose_pub;
ros::Publisher path_to_control;
ros::Publisher cloud_registered_pub;
ros::Publisher marker_pub_box;

std::mutex dataMutex;
using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

enum State
{
  SearchMode,
  ReturnMode,
  AlignMode
};

//动态保存点云地图
std::queue<pcl::PointCloud<pcl::PointXYZ>> pointcloud_map_queue;
const int queue_size = 20;
std::queue<std::pair<PointCloud, Vector3d>> pointcloud_vector3d_queue;
const int queue2_size = 10; //如果卡顿修改他
Vector3d last_point = {0,0,0};
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;


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
Vector3d start_pt = {0,0,0};
Vector3d target_pt;
geometry_msgs::PoseStamped start_pose = geometry_msgs::PoseStamped();
World* world = NULL;
Minimum_jerk mj = Minimum_jerk();
PFRRTStar* pf_rrt_star = NULL;
double max_vel;
double max_acc;

double plane_bottom;
double planning_horizon;
double planning_time_horizon; 

//single thread for pose pub

// function declaration
void rcvWaypointsCallback(const nav_msgs::Path& wp);
void rcvPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &surfmap_msg);
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub);
void findSolution();
void callPlanner();
void pubPathToControl(ros::Publisher* path_to_control_pub);
void visualBox(const geometry_msgs::PoseStamped& pose);

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

int cloud_count = 0;
// void multi_callback(const sensor_msgs::PointCloud2ConstPtr &surfmap_msg,
//                     const sensor_msgs::PointCloud2ConstPtr &cloud_registered_msg) {
void multi_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_registered_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(*surfmap_msg, *cloud);
  // std::cout<<"cloud size: "<<cloud->size()<<std::endl;
  std::cout << "callback " << "\n";
  auto start_time = std::chrono::steady_clock::now();

  sensor_msgs::PointCloud2 msg;
  msg = *cloud_registered_msg;
  msg.header.stamp = ros::Time::now();
  cloud_registered_pub.publish(msg);

  pcl::PointCloud<pcl::PointXYZ> cloud_registered;

  pcl::fromROSMsg(*cloud_registered_msg, cloud_registered);
  pointcloud_map_queue.push(cloud_registered);
  if(pointcloud_map_queue.size() > queue_size){
    pointcloud_map_queue.pop();
  }
  std::queue<pcl::PointCloud<pcl::PointXYZ>> pointcloud_map_queue_copy = pointcloud_map_queue;
  pcl::PointCloud<pcl::PointXYZ> cloud_registered_queue;
  //遍历队列,将队列中的点云合并
  while(!pointcloud_map_queue_copy.empty()){
    cloud_registered_queue += pointcloud_map_queue_copy.front();
    pointcloud_map_queue_copy.pop();
  }

  if(pointcloud_map_queue.size() >= 20) 
  {
    if(last_point.norm() == 0 || (start_pt - last_point).norm() > 0.5)
    {
      {
        std::pair<PointCloud, Vector3d> tempPair = std::make_pair(cloud_registered_queue, start_pt);
        if(pointcloud_vector3d_queue.size() > queue2_size){
          pointcloud_vector3d_queue.pop();
        }
        pointcloud_vector3d_queue.push(tempPair);
        last_point = start_pt;
      }
    }
    std::queue<std::pair<PointCloud, Vector3d>> pointcloud_vector3d_queue_copy = pointcloud_vector3d_queue;
    PointCloud cloud_registered_vector3d_queue;
    //遍历队列,将队列中的点云合并
    while(!pointcloud_vector3d_queue_copy.empty()){
      cloud_registered_vector3d_queue += pointcloud_vector3d_queue_copy.front().first;
      pointcloud_vector3d_queue_copy.pop();
    }
    *cloud += cloud_registered_vector3d_queue;
  }
  *cloud += cloud_registered_queue;

  std::cout<<"cloud_size: "<<cloud->size()<<"\n";

  //转换坐标
  Eigen::Vector3d translation(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
  Eigen::Quaterniond rotation(start_pose.pose.orientation.w, start_pose.pose.orientation.x,
                              start_pose.pose.orientation.y, start_pose.pose.orientation.z);
  Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();
  for(int i = -14; i <= 14 ; i++){
    for(int j = -10; j <= 10; j++){
      Vector3d plane = {i*0.05,j*0.05,plane_bottom};
      Vector3d plane_transformed = rotationMatrix * plane + translation;
      PointT point;
      point.x = plane_transformed(0);
      point.y = plane_transformed(1);
      point.z = plane_transformed(2);
      cloud->points.push_back(point);
    }
  }

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-9999, start_pt(2) + 2.0);
  pass.filter(*cloud);

  auto end_time1 = std::chrono::steady_clock::now();


  world->initGridMap(*cloud);
  auto end_time2 = std::chrono::steady_clock::now();

  std::for_each(std::execution::par, cloud->begin(), cloud->end(), [&](const auto& pt) {  
    Vector3d obstacle(pt.x, pt.y, pt.z);  
    world->setObs(obstacle);  
  });  

  std::for_each(std::execution::par, cloud->begin(), cloud->end(), [&](const auto& pt) {  
    Vector3d obstacle(pt.x, pt.y, pt.z);  
    world->addObs(obstacle);  
  });  

  auto time1 = std::chrono::duration_cast<std::chrono::duration<double>>(end_time1 - start_time);
  auto time2 = std::chrono::duration_cast<std::chrono::duration<double>>(end_time2 - start_time);
  ROS_WARN("time1: %f, time2: %f", time1.count(), time2.count());

  visWorld(world, &grid_map_vis_pub);
}

void rcvPoseCallback(const geometry_msgs::PoseStamped& pose)
{
  dataMutex.lock();
  start_pt << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
  start_pose = pose;
  dataMutex.unlock();
  // save vel_direction to minimum_jerk
  Eigen::Vector3d unit_vector = {1.0, 0.0, 0.0};
  Eigen::Quaterniond tmp_quaternion(pose.pose.orientation.w, 
                                    pose.pose.orientation.x, 
                                    pose.pose.orientation.y, 
                                    pose.pose.orientation.z);
  
  // 将四元数转换为旋转矩阵
  Eigen::Matrix3d rotationMatrix = tmp_quaternion.toRotationMatrix();
  // 转换到camera_init坐标系下
  // mj.start_vel = rotationMatrix * (0.01*unit_vector);
  // mj.start_acc = rotationMatrix * (0.01*unit_vector);
  // // mj.start_vel = rotationMatrix * unit_vector;
  // // mj.start_acc = rotationMatrix * unit_vector;

  geometry_msgs::PoseStamped pose_to_control;
  pose_to_control.header.frame_id = "camera_init";
  pose_to_control.header.stamp = ros::Time::now();
  pose_to_control.pose = pose.pose;
  pose_pub.publish(pose_to_control);
  visualBox(pose_to_control);

  return;
}

/**
 *@brief Linearly interpolate the generated path to meet the needs of local planning
 */
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* path_to_control)
{
  if (path_to_control == NULL)
    return;
  // Float32MultiArray msg;
  nav_msgs::Path path_to_control_msg;
  path_to_control_msg.header.frame_id = "camera_init";
  path_to_control_msg.header.stamp = ros::Time::now();
  outputFile << "当前ros时间为： " << ros::Time::now() <<  "  新收到路径，路径长度为： " << solution.size() << "\n";
  outputFile << "当前机器人三维坐标为： x: " << start_pose.pose.position.x << " y: " << start_pose.pose.position.y << " z: " << start_pose.pose.position.z; 
  for (size_t i = 0; i < solution.size(); i++)
  {
    if (i == solution.size() - 1)
    {
      geometry_msgs::PoseStamped pose; 
      pose.header.frame_id = "camera_init";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = solution[i]->position_(0);
      pose.pose.position.y = solution[i]->position_(1);
      pose.pose.position.z = solution[i]->position_(2);
      path_to_control_msg.poses.push_back(pose);
      outputFile << "第" << i << "个路径： 三维坐标为 x: " << pose.pose.position.x << " , y: " << pose.pose.position.y
                  << " , z: " << pose.pose.position.z << "\n";
    }
    else
    {
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
      Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;
      outputFile << "进行插值： "  << i << " 和 " << i+1 << "点之间" << "\n";
      for (size_t j = 0; j < interpolation_num; j++)
      {
        geometry_msgs::PoseStamped pose; 
        pose.header.frame_id = "camera_init";
        pose.header.stamp = ros::Time::now();
        Vector3d interpt = solution[i]->position_ + diff_pt * (float)j / interpolation_num;
        pose.pose.position.x = interpt(0);
        pose.pose.position.y = interpt(1);
        pose.pose.position.z = interpt(2);
        path_to_control_msg.poses.push_back(pose);
        outputFile << "     第" << j << "个插入值： 三维坐标为 x: " << pose.pose.position.x << " , y: " << pose.pose.position.y
                << " , z: " << pose.pose.position.z << "\n";
      }
    }
  }
  path_to_control->publish(path_to_control_msg);
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
  ROS_WARN("findSolution to initWithGoal");
  pf_rrt_star->initWithGoal(start_pt, target_pt);

  // Case1: The PF-RRT* can't work at when the origin can't be project to surface
  if (pf_rrt_star->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }
  // Case2: If both the origin and the target can be projected,the PF-RRT* will execute
  //       global planning and try to generate a path
  
  else if (pf_rrt_star->state() == Global)
  {
    ROS_WARN("findSolution to planner global");
    ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;
    double max_time = 100.0;

    while (solution.type_ == Path::Empty && max_time < max_initial_time)
    {
      solution = pf_rrt_star->planner(max_iter, max_time);
      max_time += 100.0;
    }
    mj.waypoints.clear();
    double dist_sum, temp_dist = 0.0;
    mj.waypoints.push_back(start_pt);
    Vector3d temp_pt = start_pt;
    ROS_WARN("solution.nodes_.size(): %d", solution.nodes_.size());
    if(solution.nodes_.size() < 2){
      if((start_pt-target_pt).norm() > 0.2) mj.waypoints.push_back(target_pt);
    }else
    {
        for (auto it = solution.nodes_.rbegin() + 1; it != solution.nodes_.rend(); ++it) {
        const auto &node = *it;
        temp_dist = (node->position_ - temp_pt).norm();
        if(temp_dist < 0.2) continue;
        double cos_theta = (node->position_ - start_pt).dot(node->position_ - target_pt);
        if(cos_theta > 0 && cos_theta < 0.3) continue;
        // dist_sum += temp_dist;
        // if(dist_sum > 5.0){
        //   dist_sum = 0.0;
        //   temp_dist = 0.0;
        //   break;
        // }
        temp_pt = node->position_;
        mj.waypoints.push_back(node->position_);
      }
    }
    

    // Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6*(mj.waypoints.size()-1), 3);
    // mj.getTimeVector(0.3,0.1); //TODO:max_vel, max_acc
    // mj.solve_minimum_jerk(mj.start_vel, mj.start_acc, coefficientMatrix);
    // mj.solve_minimum_jerk({0,0,0}, {0,0,0}, coefficientMatrix); //暂时先用零向量代替

    // file << "Time: " << ros::Time::now() << "\n";
    // file << "Start waypoints: \n";
    // for(int i = 0; i < mj.waypoints.size(); i++){
    //   file << mj.waypoints[i](0) << " " << mj.waypoints[i](1) << " " << mj.waypoints[i](2) << "\n";
    // }
    // file << "end \n";

    // file << "Start timeVector: \n";
    // for(int i = 0; i < mj.timeVector.size(); i++){
    //   file << mj.timeVector[i] << " ";
    // }
    // file << "end \n";

    // file << "Start coefficientMatrix: \n";
    // for(int i = 0; i < coefficientMatrix.rows(); i++){
    //   for(int j = 0; j < coefficientMatrix.cols(); j++){
    //     file << coefficientMatrix(i, j) << " ";
    //   }
    //   file << "\n";
    // }
    // file << "end \n";

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
    double dist_sum, temp_dist = 0.0;
    mj.waypoints.push_back(start_pt);
    Vector3d temp_pt = start_pt;
    if(solution.nodes_.size() < 2){
      mj.waypoints.push_back(target_pt);
    }else
    {
        for (auto it = solution.nodes_.rbegin() + 1; it != solution.nodes_.rend(); ++it) {
        const auto &node = *it;
        temp_dist = (node->position_ - temp_pt).norm();
        if(temp_dist < 0.1) continue;
        double cos_theta = (node->position_ - start_pt).dot(node->position_ - target_pt);
        if(cos_theta > 0 && cos_theta < 0.3) continue;
        // dist_sum += temp_dist;
        // if(dist_sum > 5.0){
        //   dist_sum = 0.0;
        //   temp_dist = 0.0;
        //   break;
        // }
        temp_pt = node->position_;
        mj.waypoints.push_back(node->position_);
      }
    }

    // Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6*(mj.waypoints.size()-1), 3);
    // mj.getTimeVector(mj.waypoints,max_vel,max_acc); //TODO:max_vel, max_acc
    // mj.solve_minimum_jerk(mj.waypoints, mj.start_vel, mj.start_acc, coefficientMatrix);
    // mj.solve_minimum_jerk(mj.waypoints, {}, {}, coefficientMatrix);

    // visTrajectory(mj.waypoints, coefficientMatrix, mj.timeVector, mj.traj_jerk_vis_pub_);

    if (!solution.nodes_.empty())
      ROS_INFO("Get a sub path!");
    else
      ROS_WARN("No solution found!");
  }
  ROS_INFO("End calling PF-RRT*");
  printf("=========================================================================\n");

  pubInterpolatedPath(solution.nodes_, &path_to_control);
  // pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution.nodes_, &path_vis_pub, start_pt);
  visSurf(solution.nodes_, &surf_vis_pub);
  // pubPathToControl(&path_to_control);

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
  /*
    按照是否有目标点进行
  */
  if (!has_goal && init_time_cost < 1000)
  {
    timeval start;
    gettimeofday(&start, NULL);
    pf_rrt_star->initWithoutGoal(start_pt); //initWithoutGoal只在这里用到 把start_pt的平面生成了
    timeval end;
    gettimeofday(&end, NULL);
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    if (pf_rrt_star->state() == WithoutGoal)
    {
      int max_iter = 550;
      double max_time = 100.0;
      pf_rrt_star->planner(max_iter, max_time);
      // ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star->tree().size()));
    }
    else if(pf_rrt_star->state() != WithoutGoal)
      ROS_WARN("The start point can't be projected,unable to execute PF-RRT* algorithm");
  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal)
  {
    ROS_WARN("callPlanner to findSolution");
    findSolution();
    init_time_cost = 0.0;
  }
  // The expansion of tree will stop after the process of initialization takes more than 1s
  // else
    // ROS_INFO("The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
}

void pubPathToControl(ros::Publisher* path_to_control_pub){
  // ROS_WARN("start_pt: %f, %f, %f", start_pt(0), start_pt(1), start_pt(2));
  if(path_to_control_pub == NULL)
    return;
  nav_msgs::Path path_to_control_msg;
  path_to_control_msg.header.frame_id = "camera_init";
  path_to_control_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < mj.waypoints.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "camera_init";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = mj.waypoints[i](0);
    pose.pose.position.y = mj.waypoints[i](1);
    pose.pose.position.z = mj.waypoints[i](2);
    path_to_control_msg.poses.push_back(pose);
  }
  path_to_control_pub->publish(path_to_control_msg);
}

void visualBox(const geometry_msgs::PoseStamped& pose){  
        visualization_msgs::MarkerArray marker_array;

        Eigen::Vector3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        Eigen::Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x,
                                    pose.pose.orientation.y, pose.pose.orientation.z);
        Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();
        Eigen::Vector3d original_vector(2,0,0);
        Eigen::Vector3d transformed_vector = rotationMatrix * original_vector + translation;


        // 创建矩形框
        visualization_msgs::Marker rectangle_marker;
        rectangle_marker.header.frame_id = pose.header.frame_id;
        rectangle_marker.ns = "rectangle";
        rectangle_marker.type = visualization_msgs::Marker::CUBE;
        rectangle_marker.action = visualization_msgs::Marker::ADD;
        rectangle_marker.scale.x = 0.01;  // 矩形框的长度
        rectangle_marker.scale.y = 2.0;  // 矩形框的高度
        rectangle_marker.scale.z = 1.0; // 矩形框的深度
        rectangle_marker.pose = pose.pose;
        rectangle_marker.pose.position.x = transformed_vector(0); 
        rectangle_marker.pose.position.y = transformed_vector(1); 
        rectangle_marker.pose.position.z = transformed_vector(2); 
        rectangle_marker.color.r = 1.0;  // 颜色为红色
        rectangle_marker.color.g = 0.0;
        rectangle_marker.color.b = 0.0;
        rectangle_marker.color.a = 1.0;  // 完全不透明

        marker_array.markers.push_back(rectangle_marker);

        marker_pub_box.publish(marker_array);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh("~");

  map_sub = nh.subscribe("map", 1, multi_callback, ros::TransportHints().tcpNoDelay());
  wp_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);
  pose_sub = nh.subscribe("/global_planning_node/robot_pose", 1, rcvPoseCallback);

  // message_filters::Subscriber<sensor_msgs::PointCloud2> surfmap_sub (nh, "map", 1000, ros::TransportHints().tcpNoDelay());
  // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_registered_sub (nh, "/cloud_registered", 1000, ros::TransportHints().tcpNoDelay());

  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
  // message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), surfmap_sub, cloud_registered_sub);  
  // sync.registerCallback(boost::bind(&multi_callback, _1, _2));
  // std::thread sync_thread([&](){
  //   sync.registerCallback(boost::bind(&multi_callback, _1, _2));
  //   ros::spin();
  // });

  grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
  path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 40);
  goal_vis_pub = nh.advertise<visualization_msgs::Marker>("goal_vis", 1);
  surf_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("surf_vis", 100);
  tree_vis_pub = nh.advertise<visualization_msgs::Marker>("tree_vis", 40);
  tree_tra_pub = nh.advertise<std_msgs::Float32MultiArray>("tree_tra", 40);
  path_interpolation_pub = nh.advertise<std_msgs::Float32MultiArray>("global_path", 1000);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robotPose", 1000);
  path_to_control = nh.advertise<nav_msgs::Path>("path_to_control", 1000);
  traj_jerk_vis_pub = nh.advertise<nav_msgs::Path>("trajectory_path", 40);
  cloud_registered_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_registered_surround", 100);
  marker_pub_box = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_box", 10);

  nh.param("map/resolution", resolution, 0.1);

  // 规划参数
  nh.param("planning/goal_thre", goal_thre, 1.0);
  nh.param("planning/step_size", step_size, 0.2);
  nh.param("planning/h_surf_car", h_surf_car, 0.1);
  nh.param("planning/neighbor_radius", neighbor_radius, 1.0);

  // 平面拟合参数
  nh.param("planning/w_fit_plane", fit_plane_arg.w_total_, 0.4);
  nh.param("planning/w_flatness", fit_plane_arg.w_flatness_, 4000.0);
  nh.param("planning/w_slope", fit_plane_arg.w_slope_, 0.4);
  nh.param("planning/w_sparsity", fit_plane_arg.w_sparsity_, 0.4);
  nh.param("planning/ratio_min", fit_plane_arg.ratio_min_, 0.25);
  nh.param("planning/ratio_max", fit_plane_arg.ratio_max_, 0.4);
  nh.param("planning/conv_thre", fit_plane_arg.conv_thre_, 0.1152);

  nh.param("planning/radius_fit_plane", radius_fit_plane, 1.0); // 拟合平面半径

  nh.param("planning/max_initial_time", max_initial_time, 1000.0); // 最大初始化时间

  nh.param("planning/max_vel", max_vel, 0.3);
  nh.param("planning/max_acc", max_acc, 0.1);

  nh.param("planning/plane_bottom", plane_bottom, 0.45);
  nh.param("planning/planning_horizon", planning_horizon, 5.0);
  nh.param("planning/planning_time_horizon", planning_time_horizon, 0.5);


  outputFile.open("/home/parallels/OAC_Planner/src/OAC_Planner/planner/log/waypoint_log.txt", std::ios::app);

  // Initialization
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
  
  while (ros::ok())
  {
    ros::spinOnce();
    callPlanner();
    ros::Duration(planning_time_horizon).sleep();
  }

  // spinnerThread.join();
  if(!ros::ok()){
    outputFile.close();
  } 

  return 0;
}
