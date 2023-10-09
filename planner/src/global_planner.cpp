#include "global_planner.h"

using namespace OAC::visualization;
using namespace OAC::planner;

void GlobalPlanner::init(ros::NodeHandle& nh)
{
      // map_sub = nh.subscribe("map", 1, pointCallback, ros::TransportHints().tcpNoDelay());
    map_sub_                = nh.subscribe
        ("map", 1, &GlobalPlanner::multi_callback, this, ros::TransportHints().tcpNoDelay());
    wp_sub_                 = nh.subscribe
        ("waypoints", 1, &GlobalPlanner::rcvWaypointsCallback, this);
    pose_sub_               = nh.subscribe
        ("/global_planning_node/robot_pose", 1, &GlobalPlanner::rcvPoseCallback, this);
    returnMode_sub_         = nh.subscribe
        ("/return_mode", 100, &GlobalPlanner::returnModeCallback, this);

    octo_map_vis_pub_       = nh.advertise<sensor_msgs::PointCloud2>
        ("grid_map_vis", 1);
    grid_map_vis_pub_       = nh.advertise<grid_map_msgs::GridMap>
        ("grid_map", 1000);
    path_vis_pub_           = nh.advertise<visualization_msgs::Marker>
        ("path_vis", 40);
    goal_vis_pub_           = nh.advertise<visualization_msgs::Marker>
        ("goal_vis", 1);
    surf_vis_pub_           = nh.advertise<sensor_msgs::PointCloud2>
        ("surf_vis", 100);
    tree_vis_pub_           = nh.advertise<visualization_msgs::Marker>
        ("tree_vis", 40);
    tree_tra_pub_           = nh.advertise<std_msgs::Float32MultiArray>
        ("tree_tra", 40);
    path_interpolation_pub_ = nh.advertise<std_msgs::Float32MultiArray>
        ("global_path", 1000);
    path_to_control_        = nh.advertise<nav_msgs::Path>
        ("path_to_control", 10);
    traj_jerk_vis_pub_      = nh.advertise<nav_msgs::Path>
        ("trajectory_path", 40);
    cloud_registered_pub_   = nh.advertise<sensor_msgs::PointCloud2>
        ("cloud_registered_surround", 100);
    marker_pub_box_         = nh.advertise<visualization_msgs::MarkerArray>
        ("visualization_marker_box", 10);

    // 平面拟合参数
    nh.param("planning/w_fit_plane", fit_plane_arg_.w_total_, 0.4);
    nh.param("planning/w_flatness", fit_plane_arg_.w_flatness_, 4000.0);
    nh.param("planning/w_slope", fit_plane_arg_.w_slope_, 0.4);
    nh.param("planning/w_sparsity", fit_plane_arg_.w_sparsity_, 0.4);
    nh.param("planning/ratio_min", fit_plane_arg_.ratio_min_, 0.25);
    nh.param("planning/ratio_max", fit_plane_arg_.ratio_max_, 0.4);
    nh.param("planning/conv_thre", fit_plane_arg_.conv_thre_, 0.1152);
    nh.param("planning/radius_fit_plane", radius_fit_plane_, 1.0); // 拟合平面半径
    nh.param("planning/max_initial_time", max_initial_time_, 1000.0); // 最大初始化时间
    

    nh.param("planning/max_vel", max_vel_, 0.3);
    nh.param("planning/max_acc", max_acc_, 0.1);

    nh.param("map/queue_size", queue_size_, 20);
    nh.param("map/queue2_size", queue2_size_, 15);
    nh.param("planning/plane_bottom", plane_bottom_, -0.45);
    nh.param("planning/planning_horizon", planning_horizon_, 5.0);
    nh.param("planning/planning_time_horizon", planning_time_horizon_, 0.5);

    // rrt_planner参数
    double goal_biased;
    double sub_goal_threshold;
    double inherit_threshold;
    // 规划参数
    nh.param("map/resolution", resolution_, 0.1);
    nh.param("planning/step_size", step_size_, 0.2);
    nh.param("planning/h_surf_car", h_surf_car_, 0.1);
    nh.param("planning/neighbor_radius", neighbor_radius_, 1.0);
    nh.param("plannning/goal_biased", goal_biased, 0.15);
    nh.param("planning/goal_thre", goal_thre_, 1.0);
    nh.param("plannning/sub_goal_threshold", sub_goal_threshold, 0.15);
    nh.param("plannning/inherit_threshold", inherit_threshold, 0.4);

    // logPlot 记录
    nh.param("run_time_log", run_time_log_, false);
    nh.param("run_time_print", run_time_print_, false);

    // 获取当前节点的包路径
    std::string package_path = ros::package::getPath("planner");

    outputFile_.open(package_path + "/log/waypoint_log.txt", std::ios::out | std::ios::trunc);
    keyPointDebug_.open(package_path + "/log/keypoint_log.txt", std::ios::out | std::ios::trunc);
    alignPointDebug_.open(package_path + "/log/alignpoint_log.txt", std::ios::out | std::ios::trunc);
    logFile_.open(package_path + "/log/plot_log.txt", std::ios::out | std::ios::trunc);

    // Initialization
    world_ = new World(resolution_);

    world_->run_time_print_ = run_time_print_;
  
    pf_rrt_star_ = new PFRRTStar(h_surf_car_, world_);

    // Set argument of PF-RRT*
    pf_rrt_star_->setGoalThre(goal_thre_);
    pf_rrt_star_->setSubGoalThre(sub_goal_threshold);
    pf_rrt_star_->setInheritThre(inherit_threshold);
    pf_rrt_star_->setFitPlaneArg(fit_plane_arg_);
    pf_rrt_star_->setNeighborRadius(neighbor_radius_);
    pf_rrt_star_->setFitPlaneRadius(radius_fit_plane_);
    pf_rrt_star_->setStepSize(step_size_);
    pf_rrt_star_->setGoalBiased(goal_biased);


    pf_rrt_star_->goal_vis_pub_ = &goal_vis_pub_;
    pf_rrt_star_->tree_vis_pub_ = &tree_vis_pub_;
    pf_rrt_star_->tree_tra_pub_ = &tree_tra_pub_;

    mj_.traj_jerk_vis_pub_ = &traj_jerk_vis_pub_;

    keyPoints_.reset(new PointCloud);

}

// 从rviz中获取得到目标点
void GlobalPlanner::rcvWaypointsCallback(const nav_msgs::Path& wp)
{
  auto t1 = std::chrono::steady_clock::now();
  if (!world_->has_map_)
    return;
  if(motionState_ == SearchMode)
  {
    target_pt_ = Vector3d(wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z);
    ROS_INFO("Receive the planning target");
    has_goal_ = true;
  }
  auto t2 = std::chrono::steady_clock::now();
  auto time_consume = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_WARN("rcvWaypointsCallback time: %f", time_consume.count());
  log_data_.rcv_waypoints_callback_time = time_consume.count();
}

// void multi_callback(const sensor_msgs::PointCloud2ConstPtr &surfmap_msg,
//                     const sensor_msgs::PointCloud2ConstPtr &cloud_registered_msg) {
void GlobalPlanner::multi_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_registered_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(*surfmap_msg, *cloud);
  // std::cout<<"cloud size: "<<cloud->size()<<std::endl;
  auto start_time = std::chrono::steady_clock::now();

  sensor_msgs::PointCloud2 msg;
  msg = *cloud_registered_msg;
  msg.header.stamp = ros::Time::now();
  cloud_registered_pub_.publish(msg);

  pcl::PointCloud<pcl::PointXYZ> cloud_registered;

  pcl::fromROSMsg(*cloud_registered_msg, cloud_registered);
  // pointcloud_map_queue_.push(cloud_registered);
  // if(pointcloud_map_queue_.size() > queue_size_){
  //   pointcloud_map_queue_.pop();
  // }
  // std::queue<pcl::PointCloud<pcl::PointXYZ>> pointcloud_map_queue_copy = pointcloud_map_queue_;
  // pcl::PointCloud<pcl::PointXYZ> cloud_registered_queue;
  // //遍历队列,将队列中的点云合并
  // while(!pointcloud_map_queue_copy.empty()){
  //   cloud_registered_queue += pointcloud_map_queue_copy.front();
  //   pointcloud_map_queue_copy.pop();
  // }

  // if(pointcloud_map_queue_.size() >= 20) 
  // {
  //   if(last_point_.norm() == 0 || (start_pt_ - last_point_).norm() > 0.5)
  //   {
  //     {
  //       std::pair<PointCloud, Vector3d> tempPair = std::make_pair(cloud_registered_queue, start_pt_);
  //       if(pointcloud_vector3d_queue_.size() > queue2_size_){
  //         pointcloud_vector3d_queue_.pop();
  //       }
  //       pointcloud_vector3d_queue_.push(tempPair);
  //       last_point_ = start_pt_;
  //     }
  //   }
  //   std::queue<std::pair<PointCloud, Vector3d>> pointcloud_vector3d_queue_copy = pointcloud_vector3d_queue_;
  //   PointCloud cloud_registered_vector3d_queue;
  //   //遍历队列,将队列中的点云合并
  //   while(!pointcloud_vector3d_queue_copy.empty()){
  //     cloud_registered_vector3d_queue += pointcloud_vector3d_queue_copy.front().first;
  //     pointcloud_vector3d_queue_copy.pop();
  //   }
  //   *cloud += cloud_registered_vector3d_queue;
  // }
  // *cloud += cloud_registered_queue;
  *cloud += cloud_registered;

  // std::cout<<"cloud_size: "<<cloud->size()<<"\n";

  //转换坐标
  Eigen::Vector3d translation(start_pose_.pose.position.x, start_pose_.pose.position.y, start_pose_.pose.position.z);
  Eigen::Quaterniond rotation(start_pose_.pose.orientation.w, start_pose_.pose.orientation.x,
                              start_pose_.pose.orientation.y, start_pose_.pose.orientation.z);
  Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();
  for(int i = -2; i <= 2; i++){
    for(int j = -2; j <= 2; j++){
      Vector3d plane = {i*0.05,j*0.05,plane_bottom_};
      // Vector3d plane_transformed = rotationMatrix * plane + translation;
      Vector3d plane_transformed = plane + translation;
      PointT point;
      point.x = plane_transformed(0);
      point.y = plane_transformed(1);
      point.z = plane_transformed(2);
      cloud->points.push_back(point);
    }
  }

  pass_.setInputCloud(cloud);
  pass_.setFilterFieldName("z");
  pass_.setFilterLimits(-9999, start_pt_(2) + 1.5);
  pass_.filter(*cloud);

  world_->initGridMap(*cloud);
  auto end_time1 = std::chrono::steady_clock::now();

  std::for_each(std::execution::par, cloud->begin(), cloud->end(), [&](const auto& pt) {  
    Vector3d obstacle(pt.x, pt.y, pt.z);  
    // world_->setObs(obstacle);
    world_->setGrid(obstacle);
  });  

  // grid_map_pcl::fromPointCloud2(*pclMsg, "elevation", gridMap);

  auto end_time2 = std::chrono::steady_clock::now();

  auto time1 = std::chrono::duration_cast<std::chrono::duration<double>>(end_time1 - start_time);
  auto time2 = std::chrono::duration_cast<std::chrono::duration<double>>(end_time2 - end_time1);
  if(run_time_print_) ROS_WARN("time1: %f s, time2: %f s", time1.count(), time2.count());

  log_data_.map_construction_time = time1.count();

  visWorld(world_, &octo_map_vis_pub_, &grid_map_vis_pub_);
  auto end_time3 = std::chrono::steady_clock::now();
  auto time_consume = std::chrono::duration_cast<std::chrono::duration<double>>(end_time3 - end_time2);
  auto total_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time3 - start_time);
  if(run_time_print_) ROS_WARN("vis_time: %f", time_consume);
  if(run_time_print_) ROS_WARN("multi_callback time: %f", total_time.count());
  log_data_.vis_time = time_consume.count();
  log_data_.multi_callback_time = total_time.count();
}

void GlobalPlanner::rcvPoseCallback(const geometry_msgs::PoseStamped& pose)
{
  auto t1 = std::chrono::steady_clock::now();
  gp_mutex_.lock();
  start_pt_ << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
  start_pose_ = pose;

  // 获取规划视野范围内的submap
  world_->setSubCenter(start_pt_);
  bool get_submap_flag;
  grid_map::Length length(planning_horizon_, planning_horizon_);
  world_->subMap_ = world_->gridMap_.getSubmap(world_->sub_map_center_, length, get_submap_flag);

  if(motionState_ == SearchMode)
  {
    if(start_pt_.norm() == 0 || (lastKeyPoint_ - start_pt_).norm() > 1.0)
    {
      keyPointDebug_ << "处于搜索模式，插入关键节点： " << "\n";
      keyPointDebug_ << "三维坐标为： x: " << start_pt_.x() << "  y: " << start_pt_.y() << "  z: " << start_pt_.z() << "\n";
      PointT point(start_pt_.x(), start_pt_.y(), start_pt_.z());
      keyPoints_->points.push_back(point);
      lastKeyPoint_ = start_pt_;
    }
  }
  gp_mutex_.unlock();
  // save vel_direction to minimum_jerk
  Eigen::Vector3d unit_vector = {1.0, 0.0, 0.0};
  Eigen::Quaterniond tmp_quaternion(pose.pose.orientation.w, 
                                    pose.pose.orientation.x, 
                                    pose.pose.orientation.y, 
                                    pose.pose.orientation.z);
  
  // 将四元数转换为旋转矩阵
  Eigen::Matrix3d rotationMatrix = tmp_quaternion.toRotationMatrix();
  // 转换到camera_init坐标系下
  // mj_.start_vel = rotationMatrix * (0.01*unit_vector);
  // mj_.start_acc = rotationMatrix * (0.01*unit_vector);
  // // mj_.start_vel = rotationMatrix * unit_vector;
  // // mj_.start_acc = rotationMatrix * unit_vector;

  geometry_msgs::PoseStamped pose_to_control;
  pose_to_control.header.frame_id = "camera_init";
  pose_to_control.header.stamp = ros::Time::now();
  pose_to_control.pose = pose.pose;
  visualBox(pose_to_control);
  auto t2 = std::chrono::steady_clock::now();
  auto time_consume = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_WARN("rcvPoseCallback time: %f", time_consume.count());
  log_data_.rcv_pose_callback_time = time_consume.count();

  return;
}
void GlobalPlanner::returnModeCallback(const std_msgs::String& msg)
{
  if(msg.data == "true") motionState_ = ReturnMode;
  else if(msg.data == "false") motionState_ = SearchMode;
}

void GlobalPlanner::alignModeCallback(const std_msgs::String& msg)
{

}

/**
 *@brief Linearly interpolate the generated path to meet the needs of local planning
 */
void GlobalPlanner::pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* path_to_control)
{
  if (path_to_control == NULL)
    return;
  // Float32MultiArray msg;
  nav_msgs::Path path_to_control_msg;
  path_to_control_msg.header.frame_id = "camera_init";
  path_to_control_msg.header.stamp = ros::Time::now();
  outputFile_ << "当前ros时间为： " << ros::Time::now() << "\n" << "新收到路径，路径长度为： " << solution.size() << "\n";
  outputFile_ << "当前机器人三维坐标为： x: " << start_pose_.pose.position.x << " y: " << start_pose_.pose.position.y << " z: " << start_pose_.pose.position.z << "\n"; 
  for (size_t i = 0; i < solution.size(); i++)
  {
    int k = solution.size() - i -1;
    outputFile_<<"i: " << k << "\n";
    if (k == 0)
    {
      geometry_msgs::PoseStamped pose; 
      pose.header.frame_id = "camera_init";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = solution[k]->position_(0);
      pose.pose.position.y = solution[k]->position_(1);
      pose.pose.position.z = solution[k]->position_(2);
      path_to_control_msg.poses.push_back(pose);
      outputFile_ << "第" << i << "个路径： 三维坐标为 x: " << pose.pose.position.x << " , y: " << pose.pose.position.y
                  << " , z: " << pose.pose.position.z << "\n";
    }
    else
    {
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[k], solution[k-1]) / 1.0);
      interpolation_num = interpolation_num > 0 ? interpolation_num : 1;
      Vector3d diff_pt = solution[k-1]->position_ - solution[k]->position_;
      outputFile_ << "进行插值： "  << i << " 和 " << i + 1 << "点之间" << "\n";
      for (size_t j = 0; j < interpolation_num; j++)
      {
        geometry_msgs::PoseStamped pose; 
        pose.header.frame_id = "camera_init";
        pose.header.stamp = ros::Time::now();
        Vector3d interpt = solution[k]->position_ + diff_pt * (float)j / interpolation_num;
        pose.pose.position.x = interpt(0);
        pose.pose.position.y = interpt(1);
        pose.pose.position.z = interpt(2);
        path_to_control_msg.poses.push_back(pose);
        outputFile_ << "     第" << j << "个插入值： 三维坐标为 x: " << pose.pose.position.x << " , y: " << pose.pose.position.y
                << " , z: " << pose.pose.position.z << "\n";
      }
    }
  }
  outputFile_ << "输出的路径的长度为： " << path_to_control_msg.poses.size() << "\n";
  path_to_control->publish(path_to_control_msg);
}


/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void GlobalPlanner::findSolution()
{
  printf("=========================================================================\n");
  ROS_INFO("Start calling PF-RRT*");
  Path solution = Path();
  ROS_WARN("findSolution to initWithGoal");
  pf_rrt_star_->initWithGoal(start_pt_, target_pt_);

  // Case1: The PF-RRT* can't work at when the origin can't be project to surface
  if (pf_rrt_star_->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }
  // Case2: If both the origin and the target can be projected,the PF-RRT* will execute
  //       global planning and try to generate a path
  
  else if (pf_rrt_star_->state() == Global)
  {
    ROS_WARN("findSolution to planner global");
    ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;
    double max_time = 50.0;

    while (solution.type_ == Path::Empty && max_time < max_initial_time_)
    {
      solution = pf_rrt_star_->planner(max_iter, max_time);
      max_time += 100.0;
    }
    mj_.waypoints.clear();
    double dist_sum, temp_dist = 0.0;
    mj_.waypoints.push_back(start_pt_);
    Vector3d temp_pt = start_pt_;
    if(run_time_print_) ROS_WARN("solution.nodes_.size(): %d", solution.nodes_.size());
    if(solution.nodes_.size() < 2){
      if((start_pt_-target_pt_).norm() > 0.2) mj_.waypoints.push_back(target_pt_);
    }else
    {
        for (auto it = solution.nodes_.rbegin() + 1; it != solution.nodes_.rend(); ++it) {
        const auto &node = *it;
        temp_dist = (node->position_ - temp_pt).norm();
        if(temp_dist < 0.2) continue;
        double cos_theta = (node->position_ - start_pt_).dot(node->position_ - target_pt_);
        if(cos_theta > 0 && cos_theta < 0.3) continue;
        // dist_sum += temp_dist;
        // if(dist_sum > 5.0){
        //   dist_sum = 0.0;
        //   temp_dist = 0.0;
        //   break;
        // }
        temp_pt = node->position_;
        mj_.waypoints.push_back(node->position_);
      }
    }
    

    // Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6*(mj_.waypoints.size()-1), 3);
    // mj_.getTimeVector(0.3,0.1); 
    // mj_.solve_minimum_jerk(mj_.start_vel, mj_.start_acc, coefficientMatrix);
    // mj_.solve_minimum_jerk({0,0,0}, {0,0,0}, coefficientMatrix); //暂时先用零向量代替

    // file << "Time: " << ros::Time::now() << "\n";
    // file << "Start waypoints: \n";
    // for(int i = 0; i < mj_.waypoints.size(); i++){
    //   file << mj_.waypoints[i](0) << " " << mj_.waypoints[i](1) << " " << mj_.waypoints[i](2) << "\n";
    // }
    // file << "end \n";

    // file << "Start timeVector: \n";
    // for(int i = 0; i < mj_.timeVector.size(); i++){
    //   file << mj_.timeVector[i] << " ";
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

    // visTrajectory(mj_.waypoints, coefficientMatrix, mj_.timeVector, mj_.traj_jerk_vis_pub_);

    if (!solution.nodes_.empty()){
      ROS_INFO("Get a global path!");
      pubInterpolatedPath(solution.nodes_, &path_to_control_);
    }
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

    solution = pf_rrt_star_->planner(max_iter, max_time);
    mj_.waypoints.clear();
    double dist_sum, temp_dist = 0.0;
    mj_.waypoints.push_back(start_pt_);
    Vector3d temp_pt = start_pt_;
    if(solution.nodes_.size() < 2){
      mj_.waypoints.push_back(target_pt_);
    }else
    {
        for (auto it = solution.nodes_.rbegin() + 1; it != solution.nodes_.rend(); ++it) {
        const auto &node = *it;
        temp_dist = (node->position_ - temp_pt).norm();
        if(temp_dist < 0.1) continue;
        double cos_theta = (node->position_ - start_pt_).dot(node->position_ - target_pt_);
        if(cos_theta > 0 && cos_theta < 0.3) continue;
        // dist_sum += temp_dist;
        // if(dist_sum > 5.0){
        //   dist_sum = 0.0;
        //   temp_dist = 0.0;
        //   break;
        // }
        temp_pt = node->position_;
        mj_.waypoints.push_back(node->position_);
      }
    }

    // Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6*(mj_.waypoints.size()-1), 3);
    // mj_.getTimeVector(mj_.waypoints,max_1: %f, time2: vel,max_acc); 
    // mj_.solve_minimum_jerk(mj_.waypoints, mj_.start_vel, mj_.start_acc, coefficientMatrix);
    // mj_.solve_minimum_jerk(mj_.waypoints, {}, {}, coefficientMatrix);

    // visTrajectory(mj_.waypoints, coefficientMatrix, mj_.timeVector, mj_.traj_jerk_vis_pub_);

    if (!solution.nodes_.empty())
    {
      ROS_INFO("Get a sub path!");
      pubInterpolatedPath(solution.nodes_, &path_to_control_);
    }
    else
      ROS_WARN("No solution found!");
  }
  ROS_INFO("End calling PF-RRT*");
  printf("=========================================================================\n");

  // pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution.nodes_, &path_vis_pub_, start_pt_);
  visSurf(solution.nodes_, &surf_vis_pub_);
  // pubPathToControl(&path_to_control);

  // When the PF-RRT* generates a short enough global path,it's considered that the robot has
  // reached the goal region.
  if (solution.type_ == Path::Global && EuclideanDistance(pf_rrt_star_->origin(), pf_rrt_star_->target()) < goal_thre_)
  {
    has_goal_ = false;
    visOriginAndGoal({}, &goal_vis_pub_);  // Passing an empty set to delete the previous display
    visPath({}, &path_vis_pub_, start_pt_);
    ROS_INFO("The Robot has achieved the goal!!!");
  }

  if (solution.type_ == Path::Empty)
    visPath({}, &path_vis_pub_, start_pt_);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void GlobalPlanner::callPlanner() // TODO: update callPlanner
{
  static double init_time_cost = 0.0;
  if (!world_->has_map_)
    return;

  // The tree will expand at a certain frequency to explore the space more fully
  /*
    按照是否有目标点进行
  */
  auto start_time = std::chrono::steady_clock::now();
  if (!has_goal_ && init_time_cost < 1000)
  {
    return;
    timeval start;
    gettimeofday(&start, NULL);
    pf_rrt_star_->initWithoutGoal(start_pt_); //initWithoutGoal只在这里用到 把start_pt的平面生成了
    timeval end;
    gettimeofday(&end, NULL);
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    if (pf_rrt_star_->state() == WithoutGoal)
    {
      int max_iter = 550;
      double max_time = 50.0;
      pf_rrt_star_->planner(max_iter, max_time);
      // ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star_->tree().size()));
    }
    else if(pf_rrt_star_->state() != WithoutGoal)
      ROS_WARN("The start point can't be projected,unable to execute PF-RRT* algorithm");
  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal_)
  {
    ROS_WARN("callPlanner to findSolution");
    findSolution();
    init_time_cost = 0.0;
  }

  // The expansion of tree will stop after the process of initialization takes more than 1s
  // else
  //   ROS_INFO("The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star_->tree().size()));

  auto end_time = std::chrono::steady_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);

  log_data_.planning_time = time.count();
}


/*
  @brief 运动模式检测，判断当前机器人处于哪种运动模式
*/
void GlobalPlanner::motionModeDetect()
{
  if(keyPoints_->points.size() != 0) kdtree_.setInputCloud(keyPoints_); // 关键点不为空，建立kdtree
  if(motionState_ == ReturnMode)
  {
    keyPointDebug_ << "处于返航模式： 提取关键点： " << "\n";
    PointT search_point(start_pt_.x(), start_pt_.y(), start_pt_.z());
    std::vector<int> indices(K);
    std::vector<float> distance(K);
    kdtree_.nearestKSearch(search_point, K, indices, distance);
    PointT target_point = keyPoints_->points[indices[0]];
    keyPointDebug_ << "提取到的关键点坐标为： x: " << target_point.x << "  y: " << target_point.y << "  z: " << target_point.z << "\n"; 
    Vector3d temp_pt(target_point.x, target_point.y, target_point.z);
    target_pt_ = Vector3d(target_point.x, target_point.y, target_point.z);
    if((target_pt_ - start_pt_).norm() < 0.3)
    {
      keyPoints_->points.erase(keyPoints_->points.begin() + indices[0]);
      keyPointDebug_ << "在kdtree中清除了该关键点" << "\n";
    }
  }
  else if(motionState_ == AlignMode)
  {
    alignPointDebug_ << "处于对齐模式：  提取对齐点： " << "\n";


  }
}

void GlobalPlanner::pubPathToControl(ros::Publisher* path_to_control_pub){
  if(path_to_control_pub == NULL)
    return;
  nav_msgs::Path path_to_control_msg;
  path_to_control_msg.header.frame_id = "camera_init";
  path_to_control_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < mj_.waypoints.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "camera_init";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = mj_.waypoints[i](0);
    pose.pose.position.y = mj_.waypoints[i](1);
    pose.pose.position.z = mj_.waypoints[i](2);
    path_to_control_msg.poses.push_back(pose);
  }
  path_to_control_pub->publish(path_to_control_msg);
}

void GlobalPlanner::visualBox(const geometry_msgs::PoseStamped& pose){  
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

        marker_pub_box_.publish(marker_array);
}

void GlobalPlanner::plotLog()
{
  if(!log_data_.updated) return;
  logFile_ << log_data_.main_loop_time << " " 
           << log_data_.map_construction_time << " " 
           << log_data_.planning_time << " "
           << log_data_.vis_time << " "
           << "\n";
  log_data_.updated = false;
}

void GlobalPlanner::exit()
{
    outputFile_.close();
    keyPointDebug_.close();
    alignPointDebug_.close();
    logFile_.close();
}

// void GlobalPlanner::visGridMap()
// {
//   while(ros::ok())
//   {
//     gp_mutex_.lock();
//     if(world_->has_map_)
//     {
//       visWorld(world_, &grid_map_vis_pub_);
//     }
//     gp_mutex_.unlock();
//   }
// }

GlobalPlanner::GlobalPlanner(/* args */)
{
}

GlobalPlanner::~GlobalPlanner()
{
}