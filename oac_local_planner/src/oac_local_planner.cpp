#include "oac_local_planner.h"
using namespace Eigen;

/*
    author: Xingke Xia email: klosexxk@gmail.com
    DWA工作流程
    1. 订阅全局路径 完成
    2. 获取局部地图及障碍物信息
    3. 订阅目标点
    4. 判断是否到达目的地
    5. 计算当前采样的速度范围(动态窗口)
    6. 遍历所有速度,生成轨迹,并计算每条轨迹的评价函数
    7. 选择评价函数最小的轨迹,并发布cmd_vel
*/

LocalPlanner::LocalPlanner(): nh_("~")
{
}

LocalPlanner::~LocalPlanner()
{
}

void LocalPlanner::init()
{
    nh_.param("ros_topic/odom_topic", odom_sub_topic_, std::string("/odom"));
    nh_.param("ros_topic/visual_map_topic", local_map_sub_topic_, std::string("/local_map"));
    nh_.param("ros_topic/global_path_topic", global_path_sub_topic_, std::string("/global_path"));

    nh_.param("hz", hz_, 10.0);
    nh_.param("target_velocity", target_velocity_, 1.0);
    nh_.param("max_velocity", max_velocity_, 1.0);
    nh_.param("min_velocity", min_velocity_, 0.0);
    nh_.param("max_yaw_rate", max_yaw_rate_, 1.0);
    nh_.param("min_yaw_rate", min_yaw_rate_, 0.0);
    nh_.param("max_acceleration", max_acceleration_, 1.0);
    nh_.param("max_deceleration", max_deceleration_, 1.0);
    nh_.param("max_d_yaw_rate", max_d_yaw_rate_, 1.0);
    nh_.param("angle_resolution", angle_resolution_, 0.1);
    nh_.param("predict_time", predict_time_, 3.0);
    nh_.param("dt", dt_, 0.1);
    nh_.param("velocity_samples", velocity_samples_, 3);
    nh_.param("yaw_rate_samples", yaw_rate_samples_, 20);
    nh_.param("dist_to_goal_th", dist_to_goal_th_, 0.1);
    nh_.param("angle_to_goal_th", angle_to_goal_th_, 0.1);
    nh_.param("h_surf_car", h_surf_car_, 0.5);

    nh_.param("use_livox_scan_as_input", use_livox_scan_as_input_, false);
    nh_.param("use_local_map_as_input", use_local_map_as_input_, false);
    nh_.param("use_scan_as_input", use_scan_as_input_, false);
    

    local_map_sub_   = nh_.subscribe
        (local_map_sub_topic_,                1, &LocalPlanner::localMapHandler, this);
    scan_sub_        = nh_.subscribe
        ("/cloud_registered",        1, &LocalPlanner::scanHandler, this);
    livox_scan_sub_  = nh_.subscribe
        ("/livox/lidar",             1, &LocalPlanner::livoxScanHandler, this);
    global_path_sub_ = nh_.subscribe
        (global_path_sub_topic_,  1, &LocalPlanner::globalPathHandler, this); 
    odom_sub_        = nh_.subscribe
        (odom_sub_topic_, 1, &LocalPlanner::poseHandler2, this);
    goal_sub_        = nh_.subscribe
        ("/goal",                    1, &LocalPlanner::goalHandler, this);

    traj_pub_        = nh_.advertise<sensor_msgs::PointCloud2>
        ("traj", 1);
    best_traj_pub_   = nh_.advertise<sensor_msgs::PointCloud2>
        ("best_traj", 1);
    obs_pub_         = nh_.advertise<sensor_msgs::PointCloud2>
        ("obs", 1);
    cmd_vel_pub_     = nh_.advertise<geometry_msgs::Twist>
        ("/cmd_vel", 100);
    bezier_curve_pub_= nh_.advertise<nav_msgs::Path>
        ("bezier_curve", 1);
    next_target_pub_ = nh_.advertise<visualization_msgs::Marker>
        ("next_target", 1);

    robot_state_.velocity_ = 0.0;
    robot_state_.yaw_rate_ = 0.0;

    outputFile.open("/home/beihang705/catkin_mpc/src/OAC_Planner/oac_local_planner/src/trajectory.txt", std::ios::out);
}

void LocalPlanner::process()
{
    ros::Rate rate(100);
    while(ros::ok())
    {
        /* 1. 订阅全局路径和位姿并设置下一个前进目标 
            1) globalPathHandler 
            2) poseHandler
            3) check_index
           2. 订阅局部地图和障碍物信息
            1) localMapHandler
            2) livoxScanHandler
           3. 订阅目标点
            1) goalHandler
           4. 判断是否到达目的地 
            1) has_reached
           5. 计算当前采样的速度范围(动态窗口) 
            1) DynamicWindowApproach
           6. 遍历所有速度,生成轨迹,并计算每条轨迹的评价函数
            1) DynamicWindowApproach
           7. 选择评价函数最小的轨迹,并发布cmd_vel
        */ 
        ros::spinOnce();

        if(!get_global_path_)
            continue;

        if(has_reached())
        {
            reset();
            int i = 0;
            while( i < 10)
            {
                pub_cmd_vel({0.0, 0.0});
                i++;
            }

        }else if(!has_reached_)
        {
            std::vector<double> bezier_samples = BezierCurveApproach();

            check_index();
            // ROS_WARN("next_target: %f, %f, %f", next_target_.x(), next_target_.y(), next_target_.z());
            // ROS_WARN("target: %f, %f, %f", goal_.x(), goal_.y(), goal_.z());
            // ROS_WARN("robot_state: %f, %f, %f, %f", robot_state_.x_, robot_state_.y_, robot_state_.z_, robot_state_.yaw_);

            double angle_to_goal = cal_2D_direction(Vector3d(robot_state_.x_, robot_state_.y_, robot_state_.z_), next_target_);
            std::vector<double> velocity_samples = DynamicWindowApproach();

            std::cout << "velocity : " << velocity_samples[0] << " yaw_rate : " << velocity_samples[1] << " - Updating\r";
            std::cout.flush(); // 刷新输出缓冲区

            pub_cmd_vel(velocity_samples);

            visual();  
        }

 
        rate.sleep();
    }
}

/*
    @brief: 获取局部地图用于避障检测
*/

void LocalPlanner::localMapHandler(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    local_map_.clearAll();
    grid_map::GridMapRosConverter::fromMessage(*msg, local_map_);
    has_local_map_ = true;
}

void LocalPlanner::scanHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // if(use_scan_as_input_) {
    //     pcl::PointCloud<pcl::PointXYZ> cloud;
    //     pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    //     pcl::fromROSMsg(*msg,cloud);
    //     for(auto r : cloud) {
    //         pcl::PointXYZ pt;
    //         pt.x = r.x;
    //         pt.y = r.y;
    //         pt.z = r.z;
    //         geometry_msgs::Pose pose;
    //         pose.position.x = r.x;
    //         pose.position.y = r.y;
    //         if(!inside_box(pt)) continue;
    //         filtered_cloud.push_back(pt);
    //         obs_list_.poses.push_back(pose);
    //     }
    //     ob_cloud_.header.frame_id = "camera_init";
    //     ob_cloud_.header.stamp = ros::Time::now();
    //     pcl::toROSMsg(filtered_cloud, ob_cloud_); 
    // }
}

void LocalPlanner::livoxScanHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    // ROS_WARN("livox scan handler");
    if(use_livox_scan_as_input_) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        obs_list_.poses.clear();
        for (auto r : msg->points) {
            pcl::PointXYZ pt;
            pt.x = r.x;
            pt.y = r.y;
            pt.z = r.z;
            if(!inside_box(pt)) continue;
            cloud.push_back(pt);
            geometry_msgs::Pose pose;
            pose.position.x = r.x;
            pose.position.y = r.y;
            obs_list_.poses.push_back(pose);
        }
        pcl::toROSMsg(cloud, ob_cloud_);
    }
}

bool LocalPlanner::inside_box(const pcl::PointXYZ& pt)
{
    if(pt.x < curr_pose_.pose.position.x - 2.5 || 
       pt.x > curr_pose_.pose.position.x + 2.5 || 
       pt.y < curr_pose_.pose.position.y - 2.5 || 
       pt.y > curr_pose_.pose.position.y + 2.5 || 
       pt.z < curr_pose_.pose.position.z - 0.15 || 
       pt.z > curr_pose_.pose.position.z + 1.0)
        return false;
    return true;
}

void LocalPlanner::goalHandler(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_WARN("goal handler");
    goal_rel_ = *msg;
    try
    {
        tf_listener_.transformPose("base_link", ros::Time(0), goal_rel_, goal_rel_.header.frame_id, goal_rel_);
    }
    catch(tf::TransformException e)
    {
        ROS_ERROR("%s", e.what());
    }

    goal_ << msg->pose.position.x, 
             msg->pose.position.y, 
             msg->pose.position.z;
    goal_quat_.x() = msg->pose.orientation.x;
    goal_quat_.y() = msg->pose.orientation.y;
    goal_quat_.z() = msg->pose.orientation.z;
    goal_quat_.w() = msg->pose.orientation.w;
    has_reached_ = false;
    curr_t_ = 0.0;
}

void LocalPlanner::globalPathHandler(const nav_msgs::Path::ConstPtr& msg)
{
    // ROS_WARN("global path handler");
    global_path_.poses.clear();
    for(auto &p : msg->poses)
    {
        global_path_.poses.push_back(p);
    }
    if(global_path_.poses.size() > 1) 
    {
        get_global_path_ = true;
    }
}

void LocalPlanner::poseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_WARN("pose handler");
    curr_pose_.pose = msg->pose;
    robot_state_.x_ = curr_pose_.pose.position.x;
    robot_state_.y_ = curr_pose_.pose.position.y;
    robot_state_.z_ = curr_pose_.pose.position.z;
    robot_state_.yaw_ = tf::getYaw(curr_pose_.pose.orientation);
    // robot_state_.velocity_ = 0.0; // TODO: 获取真实速度
    // robot_state_.yaw_rate_ = 0.0; // TODO: 获取真实角速度
}

void LocalPlanner::poseHandler2(const nav_msgs::Path::ConstPtr& msg)
{
    // ROS_WARN("pose handler");
    curr_pose_.pose = msg->poses.back().pose;
    robot_state_.x_ = curr_pose_.pose.position.x;
    robot_state_.y_ = curr_pose_.pose.position.y;
    robot_state_.z_ = curr_pose_.pose.position.z;
    robot_state_.yaw_ = tf::getYaw(curr_pose_.pose.orientation);
    // robot_state_.velocity_ = 0.0; // TODO: 获取真实速度
    // robot_state_.yaw_rate_ = 0.0; // TODO: 获取真实角速度
}

void LocalPlanner::check_index()
{
    Vector3d curr_pos(curr_pose_.pose.position.x, curr_pose_.pose.position.y, curr_pose_.pose.position.z);
    
    double goal_dist = (curr_pos - goal_).norm();
    if(goal_dist < 2.0)
    {
        has_goal_ = true;
        next_target_ = goal_;
        return;
    } 
    Vector3d closest_point;
    double closest_dist = DBL_MAX;
    int index = 0;
    for(int i = 0; i < bezier_curve_points_.size(); i++)
    {
        double t = static_cast<double>(i) / (bezier_curve_points_.size() - 1);
        if(t < curr_t_) continue;
        Vector3d point_pos = bezier_curve_points_[i];
        double dist = (point_pos - curr_pos).norm();
        if(dist < closest_dist && dist > 2.0)
        {
            closest_dist = dist;
            closest_point = point_pos;
            index = i;
            curr_t_ = t;
        }
    }
    next_target_ = closest_point;
}

double LocalPlanner::cal_point_angle(const Eigen::Vector3d &s_pt, const Eigen::Vector3d &t_pt)
{
    double angle = acos(s_pt.dot(t_pt) / (s_pt.norm() * t_pt.norm()));
    return angle;
}

bool LocalPlanner::has_reached()
{
    Vector2d curr_pos(curr_pose_.pose.position.x, curr_pose_.pose.position.y);
    Vector2d goal(goal_.x(), goal_.y());
    if((goal - curr_pos).norm() < dist_to_goal_th_)
    {
        return true;
    }
    return false;
}

void LocalPlanner::reset()
{
    has_goal_ = false;
    get_global_path_ = false;
    global_path_.poses.clear();
    obs_list_.poses.clear();
    robot_state_.velocity_ = 0.0;
    robot_state_.yaw_rate_ = 0.0;
    bezier_curve_points_.clear();
    bezier_curve_derivative_.clear();
    curr_t_ = 0.0;
    has_reached_ = true;
    traj_list_.clear();
    has_goal_ = false;
}

std::vector<double> LocalPlanner::DynamicWindowApproach()
{
    traj_list_.clear();
    std::vector<double> velocity_samples;
    Window window = calc_dynamic_window();

    std::vector<TrajectoryCost> trajectory_costs;

    // 评价指标求和
    double sum_clearance_cost = 0.0;
    double sum_heading_cost = 0.0;
    double sum_velocity_cost = 0.0;
    double sum_dist_cost = 0.0;

    if(can_adjust_robot_direction())
    {
        double angle_path_edge = cal_2D_direction(Vector3d(robot_state_.x_, robot_state_.y_, robot_state_.z_), next_target_);
        // const double angle_to_goal = cal_2D_direction(Vector3d(robot_state_.x_, robot_state_.y_, robot_state_.z_), goal_);
        double angle_to_goal = angle_path_edge - robot_state_.yaw_;
        while(fabs(angle_to_goal) > M_PI)
        {
            angle_to_goal > 0 ? angle_to_goal -= 2 * M_PI : angle_to_goal += 2 * M_PI;
        }
        double yaw_rate = angle_to_goal > 0 ? std::min(angle_to_goal, max_yaw_rate_) : std::max(angle_to_goal, min_yaw_rate_);
        outputFile << "time: " << ros::Time::now() << " yaw_rate: " << yaw_rate << \
            " angle_path_edge: " << angle_path_edge << " yaw: " << robot_state_.yaw_ << \
            " angle_to_goal: " << angle_to_goal << "\n";
    
        std::vector<State> traj = generate_trajectory(yaw_rate);
        traj_list_.push_back(traj);
        TrajectoryCost temp_cost;
        float clearance_cost = cal_clearance(traj);
        // float stop_dist = cal_breaking_dist(v);

        double heading = cal_2D_direction(Vector3d(traj.back().x_, traj.back().y_, traj.back().z_), next_target_);
        // double dist_to_goal = sqrt(pow(traj.back().x_ - next_target_.x(), 2) + pow(traj.back().y_ - next_target_.y(), 2));
        // double heading = cal_2D_direction(Vector3d(traj.back().x_, traj.back().y_, traj.back().z_), goal_);
        double dist_to_goal = sqrt(pow(traj.back().x_ - goal_.x(), 2) + pow(traj.back().y_ - goal_.y(), 2));
        // double velocity_cost = abs(target_velocity_ - traj.back().velocity_);
        double velocity_cost = 1/ abs(traj.back().velocity_ + 0.000001);
        double heading_cost = abs(heading - traj.back().yaw_);
        double dist_cost = dist_to_goal;

        temp_cost.clearance_cost_ = clearance_cost;

        temp_cost.heading_cost_ = heading_cost;
        temp_cost.velocity_cost_ = velocity_cost;
        temp_cost.dist_cost_ = dist_cost;
        temp_cost.velocity_ = 0;
        temp_cost.yaw_rate_ = yaw_rate;
        trajectory_costs.push_back(temp_cost);
        
        sum_clearance_cost += clearance_cost;
        sum_heading_cost += heading_cost;
        sum_velocity_cost += velocity_cost;
        sum_dist_cost += dist_cost;
    }
    else{
        for(double v = window.min_v_; v < window.max_v_; v += 0.02)
        {
            for(double y = window.min_yaw_rate_; y < window.max_yaw_rate_; y += 0.02)
            {

                std::vector<State> traj = generate_trajectory(v, y);
                // ROS_WARN("traj size: %d", traj.size());
                // ROS_WARN("traj.front: %f, %f, %f", traj.front().x_, traj.front().y_, traj.front().z_);
                if(!check_collision(traj))
                {
                    traj_list_.push_back(traj);

                    TrajectoryCost temp_cost;
                    float clearance_cost = cal_clearance(traj);
                    // float stop_dist = cal_breaking_dist(v);

                    double heading = cal_2D_direction(Vector3d(traj.back().x_, traj.back().y_, traj.back().z_), next_target_);
                    // double dist_to_goal = sqrt(pow(traj.back().x_ - next_target_.x(), 2) + pow(traj.back().y_ - next_target_.y(), 2));
                    // double heading = cal_2D_direction(Vector3d(traj.back().x_, traj.back().y_, traj.back().z_), goal_);
                    double dist_to_goal = sqrt(pow(traj.back().x_ - goal_.x(), 2) + pow(traj.back().y_ - goal_.y(), 2));
                    // double velocity_cost = abs(target_velocity_ - traj.back().velocity_);
                    double velocity_cost = 1/ abs(traj.back().velocity_ + 0.000001);
                    double heading_cost = abs(heading - traj.back().yaw_);
                    double dist_cost = dist_to_goal;

                    temp_cost.clearance_cost_ = clearance_cost;

                    temp_cost.heading_cost_ = heading_cost;
                    temp_cost.velocity_cost_ = velocity_cost;
                    temp_cost.dist_cost_ = dist_cost;
                    temp_cost.velocity_ = v;
                    temp_cost.yaw_rate_ = y;
                    trajectory_costs.push_back(temp_cost);
                    
                    sum_clearance_cost += clearance_cost;
                    sum_heading_cost += heading_cost;
                    sum_velocity_cost += velocity_cost;
                    sum_dist_cost += dist_cost;
                }               
            }
        }
    }


    // 选择最优轨迹
    double min_cost = DBL_MAX;
    TrajectoryCost best_traj_cost;
    for(int i = 0; i < trajectory_costs.size(); i++)
    {
        TrajectoryCost traj = trajectory_costs[i];
        traj.traj_cost_ = \
                          0.4 * traj.clearance_cost_ / sum_clearance_cost + 
                          0.3 * traj.velocity_cost_ / sum_velocity_cost + 
                          0.1 * traj.heading_cost_ / sum_heading_cost + 
                          0.2 * traj.dist_cost_ / sum_dist_cost;
        if(traj.traj_cost_ < min_cost)
        {
            min_cost = traj.traj_cost_;
            best_traj_cost = traj;
            best_traj_ = traj_list_[i];
        }
    }

    velocity_samples.push_back(best_traj_cost.velocity_);
    velocity_samples.push_back(best_traj_cost.yaw_rate_);
    return velocity_samples;
}

LocalPlanner::Window LocalPlanner::calc_dynamic_window()
{
    Window window(min_velocity_, max_velocity_, min_yaw_rate_, max_yaw_rate_);
    window.min_v_ = std::max((robot_state_.velocity_ - max_deceleration_ * dt_), min_velocity_);
    window.max_v_ = std::min((robot_state_.velocity_ + max_acceleration_ * dt_), max_velocity_);
    window.min_yaw_rate_ = std::max((robot_state_.yaw_rate_ - max_d_yaw_rate_ * dt_), min_yaw_rate_);
    window.max_yaw_rate_ = std::min((robot_state_.yaw_rate_ + max_d_yaw_rate_ * dt_), max_yaw_rate_);
    return window;
}

std::vector<LocalPlanner::State> LocalPlanner::generate_trajectory(const double& velocity, const double& yaw_rate)
{
    // const double angle_to_goal = cal_2D_direction(next_target_);
    // const double predict_time = angle_to_goal / (yaw_rate + DBL_EPSILON);
    const size_t traj_num = predict_time_ / dt_;
    State init_state = robot_state_;
    std::vector<State> traj;
    traj.resize(traj_num);
    traj[0] = init_state;
    State state = init_state; 
    for (size_t i = 1; i < traj_num; i++)
    {
       motion(state, velocity, yaw_rate);
       traj[i] = state; 
    }
    
    return traj;
}

std::vector<LocalPlanner::State> LocalPlanner::generate_trajectory(const double& yaw_rate)
{
    double angle_path_edge = cal_2D_direction(Vector3d(robot_state_.x_, robot_state_.y_, robot_state_.z_), next_target_);
    double angle_to_goal = angle_path_edge - robot_state_.yaw_;
    while(fabs(angle_to_goal) > M_PI)
    {
        angle_to_goal > 0 ? angle_to_goal -= 2 * M_PI : angle_to_goal += 2 * M_PI;
    }
    const double predict_time = angle_to_goal / (yaw_rate + DBL_EPSILON); 
    const size_t traj_num = predict_time / dt_;
    State init_state = robot_state_;
    std::vector<State> traj;
    traj.resize(traj_num);
    traj[0] = init_state;
    State state = init_state; 
    for (size_t i = 1; i < traj_num; i++)
    {
       motion(state, 0, yaw_rate);
       traj[i] = state; 
    }
    
    return traj;
}

void LocalPlanner::motion(State& state, const double velocity, const double yaw_rate)
{
    state.yaw_ += yaw_rate * dt_;
    state.x_ += state.velocity_ * cos(state.yaw_) * dt_;
    state.y_ += state.velocity_ * sin(state.yaw_) * dt_;
    grid_map::Position temp(state.x_, state.y_);
    state.z_ = 0.0;
    state.velocity_ = velocity;
    state.yaw_rate_ = yaw_rate;
}

double LocalPlanner::cal_clearance(const std::vector<State>& trajectory)
{
    double min_dist = Infinity;
    State state = trajectory.back();
    // for(const auto &state : trajectory){
    for(grid_map::CircleIterator iterator(local_map_, grid_map::Position(state.x_, state.y_), 2.0); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position position;
        local_map_.getPosition(*iterator, position);
        if(local_map_.isInside(position))
        {
            if(local_map_.atPosition("elevation", position) == NAN)
                continue;
            if(local_map_.atPosition("dilatation_barrier", position) == 1.0)
            {
                Vector2d temp(state.x_ - position(0), state.y_ - position(1));
                double dist = temp.norm();
                if(dist < min_dist)
                    min_dist = dist;
            } 
        }
    }
    // }

    
    return 1/ (min_dist + 0.0001);
}

double LocalPlanner::cal_breaking_dist(const double& velocity)
{
    double stop_dist = 0;
    double temp_velocity = velocity;
    while(temp_velocity > 0)
    {
        stop_dist += temp_velocity * dt_;
        temp_velocity -= max_deceleration_ * dt_;
    }
    return stop_dist;
}

void LocalPlanner::can_move()
{
    
}

void LocalPlanner::pub_cmd_vel(const std::vector<double>& param)
{
    geometry_msgs::Twist cmd_vel;
    // std::pair<std::vector<State>, bool> best_traj;
    // std::vector<std::pair<std::vector<State>, bool>> traj_list;
    // const size_t traj_num = velocity_samples_ * (yaw_rate_samples_ + 1);
    // traj_list.resize(traj_num);
    // const Eigen::Vector3d goal_rel = Eigen::Vector3d(goal_rel_.pose.position.x, 
    //                                                 goal_rel_.pose.position.y, 
    //                                                 goal_rel_.pose.position.z);
    // const Eigen::Quaterniond goal_quat_rel = Eigen::Quaterniond(goal_rel_.pose.orientation.w, 
    //                                                         goal_rel_.pose.orientation.x, 
    //                                                         goal_rel_.pose.orientation.y, 
    //                                                         goal_rel_.pose.orientation.z);
    cmd_vel.linear.x = param[0];
    cmd_vel.linear.y = 0.0; 
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = param[1];

    robot_state_.velocity_ = param[0];
    robot_state_.yaw_rate_ = param[1];

    cmd_vel_pub_.publish(cmd_vel);
}

bool LocalPlanner::can_adjust_robot_direction()
{   
    // return false;
    // 计算当前位置到目标点的方向角度
    double angle_path_edge = cal_2D_direction(Vector3d(robot_state_.x_, robot_state_.y_, robot_state_.z_), next_target_);
    // const double angle_to_goal = cal_2D_direction(Vector3d(robot_state_.x_, robot_state_.y_, robot_state_.z_), goal_);
    double angle_to_goal = angle_path_edge - robot_state_.yaw_;
    while(fabs(angle_to_goal) > M_PI)
    {
        angle_to_goal > 0 ? angle_to_goal -= 2 * M_PI : angle_to_goal += 2 * M_PI;
    }
    if(abs(angle_to_goal) < angle_to_goal_th_)
        return false;
    const double yawrate = std::min(std::max(angle_to_goal, min_yaw_rate_), max_yaw_rate_);
    std::vector<State> traj = generate_trajectory(0, yawrate);

    if(check_collision(traj))
        return false;
    else
        return true;
}


bool LocalPlanner::check_collision(const std::vector<State>& trajectory)
{
    for(const auto &state : trajectory)
    {
        grid_map::Position position(state.x_, state.y_);
        if(local_map_.isInside(position))
        {
            if(local_map_.atPosition("elevation", position) == NAN)
                return true;

            // 周围0.5米内有障碍物
            for(int i = -3; i < 3; i++)
            {
                for(int j = -3; j < 3; j++)
                {
                    grid_map::Position temp(state.x_ + i * 0.1, state.y_ + j * 0.1);
                    if(local_map_.atPosition("dilatation_barrier", temp) == 1.0)
                        return true;
                }
            }
        }
    }
    return false;
}

double LocalPlanner::bernstein(const int& i, const int& n, const double& t)
{
    return static_cast<double>(tgamma(n + 1) / (tgamma(i + 1) * tgamma(n - i + 1))) * pow(t, i) * pow(1 - t, n - i);
}

Vector3d LocalPlanner::bezier_curve(const std::vector<Eigen::Vector3d>& control_points, const double& t)
{
    Vector3d result(0,0,0);
    int n = control_points.size() - 1;

    for(int i = 0; i<=n; i++)
    {
        result += control_points[i] * bernstein(i, n, t);
    }
    return result;
}

Vector3d LocalPlanner::bezier_curve_derivative(const std::vector<Eigen::Vector3d>& control_points, const double& t)
{
    Vector3d result(0, 0, 0);
    int n = control_points.size() - 1;
    for (int i = 0; i < n; ++i) {
        result += n * (control_points[i + 1] - control_points[i]) * bernstein(i, n - 1, t);
    }
    return result;
}

std::vector<double> LocalPlanner::BezierCurveApproach()
{
    double num_points = 20;
    std::vector<Vector3d> controlPoints;
    for(const auto& pose:global_path_.poses)
    {
        Vector3d point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        controlPoints.push_back(point);
    }
    bezier_curve_points_.clear();
    bezier_curve_derivative_.clear();
    for(int i = 0; i < num_points; i++)
    {
        double t = static_cast<double>(i) / (num_points - 1);
        Vector3d point = bezier_curve(controlPoints, t);
        Vector3d derivative = bezier_curve_derivative(controlPoints, t); 
        bezier_curve_points_.push_back(point);
        bezier_curve_derivative_.push_back(derivative);
    } 
    double temp_dist = INFINITY;
    int index = 0;
    for(int i = 0; i < bezier_curve_points_.size(); i++)
    {
        double dis = cal_3D_dis(bezier_curve_points_[i], Eigen::Vector3d(robot_state_.x_,robot_state_.y_,robot_state_.z_));
        if(temp_dist > dis);
        {
            temp_dist = dis;
            index = i;
        }
    }
    std::vector<double> velocity_samples;
    velocity_samples.push_back(bezier_curve_derivative_[index](0));
    velocity_samples.push_back(0);
    return velocity_samples;
}

void LocalPlanner::visual()
{

    sensor_msgs::PointCloud2 traj_msg;
    pcl::PointCloud<pcl::PointXYZ> traj_cloud;
    for(auto traj:traj_list_)
    {
        for(auto state:traj)
        {
            pcl::PointXYZ pt;
            pt.x = state.x_;
            pt.y = state.y_;
            pt.z = state.z_;
            traj_cloud.push_back(pt);
        }   
    }
    pcl::toROSMsg(traj_cloud, traj_msg);
    traj_msg.header.frame_id = "camera_init";
    traj_msg.header.stamp = ros::Time::now();
    traj_pub_.publish(traj_msg);

    sensor_msgs::PointCloud2 best_traj_msg;
    pcl::PointCloud<pcl::PointXYZ> best_traj_cloud;
    for(auto state:best_traj_)
    {
        pcl::PointXYZ pt;
        pt.x = state.x_;
        pt.y = state.y_;
        pt.z = state.z_;
        best_traj_cloud.push_back(pt);
    }
    pcl::toROSMsg(best_traj_cloud, best_traj_msg);
    best_traj_msg.header.frame_id = "camera_init";
    best_traj_msg.header.stamp = ros::Time::now();
    best_traj_pub_.publish(best_traj_msg);


    if(bezier_curve_points_.size() > 0)
    {
        nav_msgs::Path curve_path;
        for(int i = 0; i < bezier_curve_points_.size(); i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "camera_init";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = bezier_curve_points_[i](0);
            pose.pose.position.y = bezier_curve_points_[i](1);
            pose.pose.position.z = bezier_curve_points_[i](2);
            curve_path.poses.push_back(pose);
        }
        curve_path.header.frame_id = "camera_init";
        curve_path.header.stamp = ros::Time::now();
        bezier_curve_pub_.publish(curve_path);
    }

    // 用marker展示next_target_
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
    marker.ns = "next_target";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point point;
    point.x = next_target_.x();
    point.y = next_target_.y();
    point.z = next_target_.z();
    marker.points.push_back(point);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    next_target_pub_.publish(marker);


    ob_cloud_.header.frame_id = "camera_init";
    ob_cloud_.header.stamp = ros::Time::now();
    if(ob_cloud_.data.size() > 0) obs_pub_.publish(ob_cloud_);
}

// 计算旋转矩阵
Eigen::Matrix3d LocalPlanner::computeRotationMatrix(const Vector3d& from, const Vector3d& to) {
    // 计算旋转轴
    Vector3d axis = from.cross(to).normalized();

    // 计算旋转角度
    double angle = std::acos(from.dot(to) / (from.norm() * to.norm()));

    // 构建旋转矩阵
    Eigen::AngleAxisd rotation(angle, axis);
    Matrix3d rotationMatrix = rotation.toRotationMatrix();

    return rotationMatrix;
}