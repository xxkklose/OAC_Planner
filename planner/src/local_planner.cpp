#include "local_planner.h"

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

    local_map_sub_ = nh_.subscribe("local_map", 1, &LocalPlanner::localMapHandler, this);
    livox_scan_sub_ = nh_.subscribe("/livox/lidar", 1, &LocalPlanner::livoxScanHandler, this);
    goal_sub_ = nh_.subscribe("/goal", 1, &LocalPlanner::goalHandler, this);
}

void LocalPlanner::process()
{
    ros::Rate rate(hz_);
    while(ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        

        ros::spinOnce();
        rate.sleep();
    }
}

void LocalPlanner::localMapHandler(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    // ROS_WARN("local map handler");
}

void LocalPlanner::livoxScanHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    // ROS_WARN("livox scan handler");
    if(use_livox_scan_as_input_) {
        obs_list_.poses.clear();
        for (auto r : msg->points) {
            geometry_msgs::Pose pose;
            pose.position.x = r.x;
            pose.position.y = r.y;
            obs_list_.poses.push_back(pose);
        }
    }
}

void LocalPlanner::goalHandler(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_WARN("goal handler");
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
    std::vector<State> traj = generate_trajectory(yawrate, goal, goal_quat);

    return true;
}

std::vector<LocalPlanner::State> LocalPlanner::generate_trajectory(const double yaw_rate, const Eigen::Vector3d& goal, const Eigen::Quaterniond& goal_quat)
{
    const double angle_to_goal = cal_2D_direction(goal);
    const double predict_time = angle_to_goal / (yaw_rate + DBL_EPSILON);
    const size_t traj_num = predict_time_ / dt_;
    std::vector<State> traj;
    traj.resize(traj_num);
    State state; // 默认值怎么处理?
    for (size_t i = 0; i < traj_num; i++)
    {
       motion(state, 0.0, yaw_rate);
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
    state.z_ = 0.0;
    state.velocity_ = velocity;
    state.yaw_rate_ = yaw_rate;
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