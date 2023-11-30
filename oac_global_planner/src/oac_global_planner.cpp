#include "oac_global_planner.h"

using namespace OAC::visualization;
using namespace OAC::planner;

GlobalPlanner::GlobalPlanner(/* args */)
{
}

GlobalPlanner::~GlobalPlanner()
{
}

void GlobalPlanner::init(ros::NodeHandle& nh)
{
    // ros topic
    nh.param("ros_topic/odom_topic", pose_sub_topic_, std::string("/global_planning_node/robot_pose")); // fast_lio 的位姿
    nh.param("ros_topic/map_topic", map_sub_topic_, std::string("/map")); 
    nh.param("ros_topic/local_map_topic", local_map_sub_topic_, std::string("/local_map")); 
    
    map_sub_                = nh.subscribe
        (map_sub_topic_, 1, &GlobalPlanner::mapCallback, this, ros::TransportHints().tcpNoDelay());
    local_map_sub_          = nh.subscribe
        (local_map_sub_topic_, 1, &GlobalPlanner::localMapCallback, this, ros::TransportHints().tcpNoDelay());
    wp_sub_                 = nh.subscribe
        ("waypoints", 1, &GlobalPlanner::rcvWaypointsCallback, this);
    pose_sub_               = nh.subscribe
        (pose_sub_topic_, 1, &GlobalPlanner::rcvPoseCallback, this);
    returnMode_sub_         = nh.subscribe
        ("/return_mode", 100, &GlobalPlanner::returnModeCallback, this);

    global_path_pub_        = nh.advertise<nav_msgs::Path>
        ("global_path", 1);
    grid_map_vis_pub_       = nh.advertise<grid_map_msgs::GridMap>
        ("grid_map", 1);
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
    std::string package_path = ros::package::getPath("oac_global_planner");

    outputFile_.open(package_path + "/log/waypoint_log.txt", std::ios::out | std::ios::trunc);
    keyPointDebug_.open(package_path + "/log/keypoint_log.txt", std::ios::out | std::ios::trunc);
    alignPointDebug_.open(package_path + "/log/alignpoint_log.txt", std::ios::out | std::ios::trunc);
    logFile_.open(package_path + "/log/plot_log.txt", std::ios::out | std::ios::trunc);

    // Initialization
    world_ = new World(resolution_, nh);

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

    pf_rrt_star_->tree_in_Debug_.open(package_path + "/log/tree_log.txt", std::ios::out | std::ios::trunc);

    pf_rrt_star_->goal_vis_pub_ = &goal_vis_pub_;
    pf_rrt_star_->tree_vis_pub_ = &tree_vis_pub_;
    pf_rrt_star_->tree_tra_pub_ = &tree_tra_pub_;

    keyPoints_.reset(new PointCloud);

}


void GlobalPlanner::process()
{
  // std::thread visualization_thread(&GlobalPlanner::visGridMap, gp);
  ros::Rate rate(10);
  while (ros::ok())
  {
    auto spinOnce_start_time = std::chrono::steady_clock::now();
    ros::spinOnce();
    auto spinOnce_end_time = std::chrono::steady_clock::now();

    auto motionModeDetect_start_time = std::chrono::steady_clock::now();
    motionModeDetect();
    auto motionModeDetect_end_time = std::chrono::steady_clock::now();

    auto callPlanner_start_time = std::chrono::steady_clock::now();
    callPlanner();
    auto callPlanner_end_time = std::chrono::steady_clock::now();

    auto total_time = std::chrono::duration_cast<std::chrono::duration<double>>(callPlanner_end_time - spinOnce_start_time);
    if(total_time.count() > 1e-3 && run_time_print_)
    {
      ROS_WARN("main loop time: %f", total_time.count());
      ROS_WARN("spinOnce time: %f", std::chrono::duration_cast<std::chrono::duration<double>>(spinOnce_end_time - spinOnce_start_time).count());
      ROS_WARN("motionModeDetect time: %f", std::chrono::duration_cast<std::chrono::duration<double>>(motionModeDetect_end_time - motionModeDetect_start_time).count());
      ROS_WARN("callPlanner time: %f", std::chrono::duration_cast<std::chrono::duration<double>>(callPlanner_end_time - callPlanner_start_time).count());
      log_data_.main_loop_time = total_time.count();
      log_data_.updated = true;
    }
    if(run_time_log_)
      plotLog();
    
    rate.sleep();
  }
  if(!ros::ok())
    exit();
}


void GlobalPlanner::mapCallback(const grid_map_msgs::GridMap& map_msg)
{
  auto start_time = std::chrono::steady_clock::now();
  grid_map::GridMapRosConverter::fromMessage(map_msg, world_->gridMap_);
  auto end_time1 = std::chrono::steady_clock::now();
}

void GlobalPlanner::localMapCallback(const grid_map_msgs::GridMap& map_msg)
{
  auto start_time = std::chrono::steady_clock::now();
  grid_map::GridMapRosConverter::fromMessage(map_msg, world_->subMap_);
  world_->initGridMap(world_->subMap_);
  auto end_time1 = std::chrono::steady_clock::now();
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
    pf_rrt_star_->initWithGoaled = false;
    // 设置出发点
    planning_start_pt_ = start_pt_;
    compare_path_.nodes_.clear();
    solution_.nodes_.clear();
  }
  auto t2 = std::chrono::steady_clock::now();
  auto time_consume = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_WARN("rcvWaypointsCallback time: %f", time_consume.count());
  log_data_.rcv_waypoints_callback_time = time_consume.count();
}

void GlobalPlanner::rcvPoseCallback(const nav_msgs::Path& pose)
{
  auto t1 = std::chrono::steady_clock::now();
  gp_mutex_.lock();
  // start_pt_ << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
  start_pt_ << pose.poses.back().pose.position.x, pose.poses.back().pose.position.y, pose.poses.back().pose.position.z;
  start_pose_ = pose.poses.back();

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

  auto t2 = std::chrono::steady_clock::now();
  auto time_consume = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  log_data_.rcv_pose_callback_time = time_consume.count();
}

void GlobalPlanner::returnModeCallback(const std_msgs::String& msg)
{
  if(msg.data == "true") motionState_ = ReturnMode;
  else if(msg.data == "false") motionState_ = SearchMode;
}

/**
 *@brief Linearly interpolate the generated path to meet the needs of local planning
 */
void GlobalPlanner::pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* global_path_pub)
{
  if (global_path_pub == NULL)
    return;
  // Float32MultiArray msg;
  nav_msgs::Path path_to_control_msg;
  path_to_control_msg.header.frame_id = "camera_init";
  path_to_control_msg.header.stamp = ros::Time::now();
  outputFile_ << "===========================================start=============================================\n";
  outputFile_ << "当前ros时间为： " << ros::Time::now() << "\n" << "新收到路径，路径长度为： " << solution.size() << "\n";
  outputFile_ << "当前机器人三维坐标为： x: " << start_pose_.pose.position.x << " y: " << start_pose_.pose.position.y << " z: " << start_pose_.pose.position.z << "\n"; 
  for (size_t i = 0; i < solution.size(); i++)
  {
    int k = solution.size() - i -1;
    outputFile_<<"i: " << i << "\n";
    if (i == solution.size() - 1)
    {
      geometry_msgs::PoseStamped pose; 
      pose.header.frame_id = "camera_init";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = solution[i]->position_(0);
      pose.pose.position.y = solution[i]->position_(1);
      pose.pose.position.z = solution[i]->position_(2);
      path_to_control_msg.poses.push_back(pose);
      outputFile_ << "第" << i << "个路径点： 三维坐标为 x: " << pose.pose.position.x << " , y: " << pose.pose.position.y
                  << " , z: " << pose.pose.position.z << "\n";
    }
    else
    {
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[k], solution[k-1]) / 1.0);
      interpolation_num = interpolation_num > 0 ? interpolation_num : 1;
      Vector3d diff_pt = solution[i]->position_ - solution[i+1]->position_;
      outputFile_ << "进行插值： "  << i << " 和 " << i + 1 << "点之间" << "\n";
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
        outputFile_ << "     第" << j << "个插入值： 三维坐标为 x: " << pose.pose.position.x << " , y: " << pose.pose.position.y
                << " , z: " << pose.pose.position.z << "\n";
      }
    }
  }
  outputFile_ << "输出的路径的长度为： " << path_to_control_msg.poses.size() << "\n";
  outputFile_ << "===========================================end=============================================\n";
  global_path_pub->publish(path_to_control_msg);
}

void GlobalPlanner::removeGlobalPathPoints(Path& solution)
{
  double temp_dist = 0.0;
  Vector3d temp_pt = solution.nodes_.back()->position_;
  for (auto it = solution.nodes_.begin(); it != solution.nodes_.end(); ) 
  {
    if(it != solution.nodes_.begin() && it != solution.nodes_.end() - 1){
      temp_dist = ((*it)->position_ - temp_pt).norm();
      double cos_theta = ((*it)->position_ - planning_start_pt_).dot((*it)->position_ - target_pt_);
      if(temp_dist < 0.2)
      {
        it = solution.nodes_.erase(it);
        continue;
      }else if(cos_theta > 0 && cos_theta < 0.3)
      {
        it = solution.nodes_.erase(it);
        continue;
      }
    }
    ++it;
  }  
}

bool GlobalPlanner::needChangeGlobalPath(const Path &solution)
{
  if(compare_path_.nodes_.size() == 0)
  {
    compare_path_.updatePath(solution);
    return true;
  }
  else
  {
    // 如果存在碰撞,需要更新
    if(!checkSolutionCollision())
    {
      solution_.updatePath(solution);
      return true;
    }
    // cost更小,需要更新 (但是solution_采用10次稳定的路径才更新)
    if(compare_path_.cost_ > solution.cost_)  
    {
      compare_path_.updatePath(solution);
      solution_not_change_count_ = 0;
      return true;
    }
    else solution_not_change_count_++;
    if(solution_not_change_count_ > 20)
    {
      solution_.updatePath(compare_path_);
      solution_not_change_count_ = 0;
      return true;
    }
  }
  return false;
}

bool GlobalPlanner::checkSolutionCollision()
{
  if(solution_.nodes_.size() == 0)
    return true;
  for(int i = 0; i < solution_.nodes_.size() - 1; i++)
  {
    // 存在碰撞
    if(!world_->collisionFree(solution_.nodes_[i], solution_.nodes_[i + 1]))
      return false; 
  }
  return true;
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void GlobalPlanner::findSolution()
{
  // printf("=========================================================================\n");
  // ROS_INFO("Start calling PF-RRT*");
  Path solution = Path();

  if(!pf_rrt_star_->initWithGoaled)
    pf_rrt_star_->initWithGoal(planning_start_pt_, target_pt_);

  // Case1: The PF-RRT* can't work at when the origin can't be project to surface
  if (pf_rrt_star_->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }
  // Case2: If both the origin and the target can be projected,the PF-RRT* will execute
  //       global planning and try to generate a path
  
  else if (pf_rrt_star_->state() == Global)
  {
    // ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;
    double max_time = 100.0;

    while (solution.type_ == Path::Empty && max_time < max_initial_time_)
    {
      solution = pf_rrt_star_->planner(max_iter, max_time);
      max_time += 100.0;
    }

    std::reverse(solution.nodes_.begin(), solution.nodes_.end());

    removeGlobalPathPoints(solution);

    if (!solution.nodes_.empty()){
      // ROS_INFO("Get a global path!");
      gp_mutex_.lock();
      needChangeGlobalPath(solution);
      gp_mutex_.unlock();
      if(solution_.nodes_.size() > 0)
        pubInterpolatedPath(solution_.nodes_, &global_path_pub_);
    }
    else
      ROS_WARN("No solution found!");
  }
  // Case3: If the origin can be projected while the target can not,the PF-RRT*
  //       will try to find a temporary target for transitions.
  else
  {
    // ROS_INFO("Starting PF-RRT* algorithm at the state of rolling planning");
    int max_iter = 1500;
    double max_time = 100.0;

    solution = pf_rrt_star_->planner(max_iter, max_time);

    std::reverse(solution.nodes_.begin(), solution.nodes_.end());

    removeGlobalPathPoints(solution);

    if (!solution.nodes_.empty())
    {
      // ROS_INFO("Get a sub path!");
      gp_mutex_.lock();
      needChangeGlobalPath(solution);
      gp_mutex_.unlock();
      if(solution_.nodes_.size() > 0)
        pubInterpolatedPath(solution_.nodes_, &global_path_pub_);
    }
    else
      ROS_WARN("No solution found!");
  }
  // ROS_INFO("End calling PF-RRT*");
  // printf("=========================================================================\n");

  // pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution_.nodes_, &path_vis_pub_, planning_start_pt_);
  visSurf(solution_.nodes_, &surf_vis_pub_);
  // pubPathToControl(&global_path_pub);

  // When the PF-RRT* generates a short enough global path,it's considered that the robot has
  // reached the goal region.
  if (solution_.type_ == Path::Global && EuclideanDistance(start_pt_, pf_rrt_star_->target()->position_) < goal_thre_)
  {
    has_goal_ = false;
    compare_path_.nodes_.clear();
    solution_.nodes_.clear();
    visOriginAndGoal({}, &goal_vis_pub_);  // Passing an empty set to delete the previous display
    visPath({}, &path_vis_pub_, planning_start_pt_);
    ROS_INFO("The Robot has achieved the goal!!!");
  }

  if (solution_.type_ == Path::Empty)
    visPath({}, &path_vis_pub_, planning_start_pt_);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void GlobalPlanner::callPlanner()
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
    // return;
    timeval start;
    gettimeofday(&start, NULL);
    pf_rrt_star_->initWithoutGoal(planning_start_pt_); //initWithoutGoal只在这里用到 把start_pt的平面生成了
    
    timeval end;
    gettimeofday(&end, NULL);
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    if (pf_rrt_star_->state() == WithoutGoal)
    {
      int max_iter = 550;
      double max_time = 100.0;
      pf_rrt_star_->planner(max_iter, max_time);

      // ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star_->tree().size()));
    }
    else if(pf_rrt_star_->state() != WithoutGoal)
      ROS_WARN("The start point can't be projected,unable to execute PF-RRT* algorithm");

  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal_)
  {
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

