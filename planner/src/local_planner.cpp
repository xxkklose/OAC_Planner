#include "local_planner.h"
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

    nh_.param("use_livox_scan_as_input", use_livox_scan_as_input_, false);
    nh_.param("use_local_map_as_input", use_local_map_as_input_, false);
    nh_.param("use_scan_as_input", use_scan_as_input_, false);
    

    local_map_sub_   = nh_.subscribe
        ("local_map",                1, &LocalPlanner::localMapHandler, this);
    scan_sub_        = nh_.subscribe
        ("/cloud_registered",        1, &LocalPlanner::scanHandler, this);
    livox_scan_sub_  = nh_.subscribe
        ("/livox/lidar",             1, &LocalPlanner::livoxScanHandler, this);
    global_path_sub_ = nh_.subscribe
        ("/global_planning_node/path_to_local",  1, &LocalPlanner::globalPathHandler, this); 
    pose_sub_        = nh_.subscribe
        ("/", 1, &LocalPlanner::poseHandler, this);
        // ("/global_planning_node/robot_pose", 1, &LocalPlanner::poseHandler, this);
    goal_sub_        = nh_.subscribe
        ("/goal",                    1, &LocalPlanner::goalHandler, this);

    traj_pub_        = nh_.advertise<sensor_msgs::PointCloud2>
        ("traj", 1);
    obs_pub_         = nh_.advertise<sensor_msgs::PointCloud2>
        ("obs", 1);
}

void LocalPlanner::process()
{
    ros::Rate rate(hz_);
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

        check_index();

        if(has_reached()) reset();

        std::vector<double> velocity_samples = DynamicWindowApproach();

        // ROS_WARN("velocity: %f, yaw_rate: %f", velocity_samples[0], velocity_samples[1]);
        
        visual();   

        geometry_msgs::Twist cmd_vel;

        // rate.sleep();
    }
}

void LocalPlanner::localMapHandler(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    // ROS_WARN("local map handler");
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
}

void LocalPlanner::globalPathHandler(const nav_msgs::Path::ConstPtr& msg)
{
    // ROS_WARN("global path handler");
    global_path_.poses.clear();
    for(auto &p : msg->poses)
    {
        global_path_.poses.push_back(p);
    }
    if(global_path_.poses.size() > 0) get_global_path_ = true;
}

void LocalPlanner::poseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_WARN("pose handler");
    curr_pose_.pose = msg->pose;
    robot_state_.x_ = curr_pose_.pose.position.x;
    robot_state_.y_ = curr_pose_.pose.position.y;
    robot_state_.z_ = curr_pose_.pose.position.z;
    robot_state_.yaw_ = tf::getYaw(curr_pose_.pose.orientation);
    robot_state_.velocity_ = 0.0; // TODO: 获取真实速度
    robot_state_.yaw_rate_ = 0.0; // TODO: 获取真实角速度
}

void LocalPlanner::check_index()
{
    Vector3d curr_pos(curr_pose_.pose.position.x, curr_pose_.pose.position.y, curr_pose_.pose.position.z);
    Vector3d closest_point;
    double closest_dist = DBL_MAX;
    int index = 0;
    for(int i = 0; i < global_path_.poses.size(); i++)
    {
        Vector3d point_pos(global_path_.poses[i].pose.position.x, global_path_.poses[i].pose.position.y, global_path_.poses[i].pose.position.z);
        double dist = (point_pos - curr_pos).norm();
        if(dist < closest_dist && dist > 0.2)
        {
            closest_dist = dist;
            closest_point = point_pos;
            index = i;
        }
    }
    // 判断朝向
    if(index == global_path_.poses.size() - 1)
    {
        next_target_ = closest_point;
    }else
    {
        Vector3d next_point(global_path_.poses[index + 1].pose.position.x, global_path_.poses[index + 1].pose.position.y, global_path_.poses[index + 1].pose.position.z);
        Vector3d edge1 = closest_point - curr_pos;
        Vector3d edge2 = next_point - curr_pos;
        double angle = cal_point_angle(edge1, edge2);
        if(angle > M_PI / 2)
            next_target_ = next_point;
        else
            next_target_ = closest_point;
    }
    has_goal_ = true;
}

double LocalPlanner::cal_point_angle(const Eigen::Vector3d &s_pt, const Eigen::Vector3d &t_pt)
{
    double angle = acos(s_pt.dot(t_pt) / (s_pt.norm() * t_pt.norm()));
    return angle;
}

bool LocalPlanner::has_reached()
{
    Vector3d curr_pos(curr_pose_.pose.position.x, curr_pose_.pose.position.y, curr_pose_.pose.position.z);
    Vector3d goal(goal_.x(), goal_.y(), goal_.z());
    if((goal - curr_pos).norm() < dist_to_goal_th_)
    {
        return true;
    }
    return false;
}

void LocalPlanner::reset()
{}

std::vector<double> LocalPlanner::DynamicWindowApproach()
{
    traj_list_.clear();
    std::vector<double> velocity_samples;
    Window window = calc_dynamic_window();

    std::vector<TrajectoryCost> trajectory_costs;

    // 评价指标求和
    double sum_clearance_cost = 0.0;
    double sum_heading_cost = 0.0;
    double sum_dist_cost = 0.0;

    for(double v = window.min_v_; v < window.max_v_; v += 0.02)
    {
        for(double y = window.min_yaw_rate_; y < window.max_yaw_rate_; y += 0.02)
        {
            std::vector<State> traj = generate_trajectory(v, y);
            traj_list_.push_back(traj);
            // if(!check_collision(traj))
            // {
            TrajectoryCost temp_cost;
            float clearance_cost = cal_clearance(traj);
            float stop_dist = cal_breaking_dist(v);

            // if(stop_dist < clearance_cost)
            // {
            double heading = cal_2D_direction(Vector3d(traj.back().x_, traj.back().y_, traj.back().z_), next_target_);
            double dist_to_goal = sqrt(pow(traj.back().x_ - next_target_.x(), 2) + pow(traj.back().y_ - next_target_.y(), 2));
            
            // double velocity_cost = abs(target_velocity_ - traj.back().velocity_);
            double heading_cost = abs(heading - traj.back().yaw_);
            double dist_cost = dist_to_goal;

            temp_cost.clearance_cost_ = clearance_cost;
            temp_cost.heading_cost_ = heading_cost;
            temp_cost.dist_cost_ = dist_cost;
            temp_cost.velocity_ = v;
            temp_cost.yaw_rate_ = y;
            trajectory_costs.push_back(temp_cost);
            
            sum_clearance_cost += clearance_cost;
            sum_heading_cost += heading_cost;
            sum_dist_cost += dist_cost;
            // }                
        }
    }

    // 选择最优轨迹
    double min_cost = DBL_MAX;
    TrajectoryCost best_traj;
    for(auto &traj : trajectory_costs)
    {
        traj.traj_cost_ = 0.3 * traj.clearance_cost_ / sum_clearance_cost + 
                          0.2 * traj.heading_cost_ / sum_heading_cost + 
                          0.5 * traj.dist_cost_ / sum_dist_cost;
        if(traj.traj_cost_ < min_cost)
        {
            min_cost = traj.traj_cost_;
            best_traj = traj;
        }
    }

    velocity_samples.push_back(best_traj.velocity_);
    velocity_samples.push_back(best_traj.yaw_rate_);
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
    traj.push_back(init_state);
    State state = init_state; 
    for (size_t i = 1; i < traj_num; i++)
    {
       motion(state, velocity, yaw_rate);
       traj[i] = state; 
    }
    
    return traj;
}

void LocalPlanner::motion(State& state, const double velocity, const double yaw_rate)
{
    state.yaw_ += yaw_rate * dt_;
    state.x_ += state.velocity_ * cos(state.yaw_) * dt_;
    state.y_ += state.velocity_ * sin(state.yaw_) * dt_;
    // TODO: z轴从local_map中获取
    state.z_ = 0.1;
    state.velocity_ = velocity;
    state.yaw_rate_ = yaw_rate;
}

double LocalPlanner::cal_clearance(const std::vector<State>& trajectory)
{
    double min_dist = DBL_MAX;
    State state = trajectory.back();
    for(const auto &obs : obs_list_.poses)
    {
        double dist = sqrt(pow(state.x_ - obs.position.x, 2) + pow(state.y_ - obs.position.y, 2));
        dist -= 0.5; //TODO: 机器人半径
        if(dist < min_dist)
            min_dist = dist;
    }
    
    return min_dist;
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

geometry_msgs::Twist LocalPlanner::cal_cmd_vel()
{
    geometry_msgs::Twist cmd_vel;
    std::pair<std::vector<State>, bool> best_traj;
    std::vector<std::pair<std::vector<State>, bool>> traj_list;
    const size_t traj_num = velocity_samples_ * (yaw_rate_samples_ + 1);
    traj_list.resize(traj_num);
    const Eigen::Vector3d goal_rel = Eigen::Vector3d(goal_rel_.pose.position.x, 
                                                    goal_rel_.pose.position.y, 
                                                    goal_rel_.pose.position.z);
    const Eigen::Quaterniond goal_quat_rel = Eigen::Quaterniond(goal_rel_.pose.orientation.w, 
                                                            goal_rel_.pose.orientation.x, 
                                                            goal_rel_.pose.orientation.y, 
                                                            goal_rel_.pose.orientation.z);

    // 线程锁操作goal_
    mutex_.lock();
    if (dist_to_goal_th_ < goal_.norm() && !has_reached_) { // 未到达目标点
        if(can_adjust_robot_direction(goal_rel, goal_quat_rel)) {
        }

    }
    else
    {
        has_reached_ = true;
    }
    mutex_.unlock();

    return cmd_vel;
}

bool LocalPlanner::can_adjust_robot_direction(const Eigen::Vector3d& goal, const Eigen::Quaterniond& goal_quat)
{   
    // 计算当前位置到目标点的方向角度
    const double angle_to_goal = cal_2D_direction(goal);
    if(abs(angle_to_goal) < angle_to_goal_th_)
        return false;
    // 在xs内
    const double yawrate = std::min(std::max(angle_to_goal, min_yaw_rate_), max_yaw_rate_);
    // std::vector<State> traj = generate_trajectory(yawrate, goal, goal_quat);

    return true;
}


bool LocalPlanner::check_collision(const std::vector<State>& trajectory)
{
    for(const auto &state : trajectory)
    {
        for(const auto &obs : obs_list_.poses)
        {
            const double dist = sqrt(pow(state.x_ - obs.position.x, 2) + pow(state.y_ - obs.position.y, 2));
            if(dist < 0.1)
                return true;
        }
    }
    return false;
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

    ob_cloud_.header.frame_id = "camera_init";
    ob_cloud_.header.stamp = ros::Time::now();
    if(ob_cloud_.data.size() > 0) obs_pub_.publish(ob_cloud_);
}