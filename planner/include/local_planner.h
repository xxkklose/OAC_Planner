#pragma once

#include "backward.hpp"
#include "planner_classes.h"
#include "livox_ros_driver/CustomMsg.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class LocalPlanner{
    public:
        LocalPlanner();
        ~LocalPlanner();

        class State
        {
            public:
                State(void) : x_(0.0), y_(0.0), z_(0.0), yaw_(0.0), velocity_(0.0), yaw_rate_(0.0){};
                ~State(){};

                State(const double x, const double y, const double z, const double yaw, const double velocity, const double yaw_rate);

                double x_;
                double y_;
                double z_;
                double yaw_;
                double velocity_;
                double yaw_rate_;
        };

        // 局部规划类参数初始化
        void init();
        void process();
        void localMapHandler(const grid_map_msgs::GridMap::ConstPtr& msg);
        void livoxScanHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg);
        void goalHandler(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void can_move();
        bool can_adjust_robot_direction(const Eigen::Vector3d & goal, const Eigen::Quaterniond& goal_quat);
        geometry_msgs::Twist cal_cmd_vel();

        // 根据角速度、目标点、目标点四元数生成轨迹
        std::vector<State> \
        generate_trajectory(const double yaw_rate, const Eigen::Vector3d& goal, const Eigen::Quaterniond& goal_quat);
        // 按照给定速度和角速度运动
        void motion(State& state, const double velocity, const double yaw_rate);
        // 碰撞检测
        bool check_collision(const std::vector<State>& trajectory);

        // 常用方法
        // 计算2维平面方向
        inline double cal_2D_direction(const Eigen::Vector3d& rel_pt){
            return atan2(rel_pt.y(), rel_pt.x());
        };
        inline double cal_2D_direction(const Eigen::Vector3d& s_pt, const Eigen::Vector3d& t_pt)
        {return atan2(t_pt.y() - s_pt.y(), t_pt.x() - s_pt.x());};
        // 计算3维方向
        inline double cal_3D_direction(const Eigen::Vector3d& s_pt, const Eigen::Vector3d& t_pt)
        {return atan2(t_pt.z() - s_pt.z(), sqrt(pow(t_pt.x() - s_pt.x(), 2) + pow(t_pt.y() - s_pt.y(), 2)));};

    protected:
        // 规划参数
        double hz_;
        double target_velocity_;
        double max_velocity_;
        double min_velocity_;
        double max_yaw_rate_;
        double min_yaw_rate_;
        double max_acceleration_;
        double max_deceleration_;
        double max_d_yaw_rate_;
        double angle_resolution_;
        double predict_time_;
        double dt_;
        int velocity_samples_; // 线速度采样点数
        int yaw_rate_samples_; // 角速度采样点数
        Eigen::Vector3d goal_; // 目标点
        Eigen::Quaterniond goal_quat_; // 目标点四元数
        geometry_msgs::PoseStamped goal_rel_; // 目标点相对当前位置位姿
        tf::TransformListener tf_listener_; // tf监听器

        double dist_to_goal_th_; // 到达目标点距离阈值
        double angle_to_goal_th_; // 到达目标点角度阈值
        bool has_reached_ = false; // 是否到达目标点

        // 条件标志
        bool use_livox_scan_as_input_; // 使用livox 扫描帧作为输入
        bool use_local_map_as_input_; // 使用grid_map局部地图作为输入
        bool use_scan_as_input_; // 使用激光雷达扫描帧作为输入

        // 局部地图障碍
        geometry_msgs::PoseArray obs_list_;

        std::mutex mutex_;

        // ros相关
        ros::NodeHandle nh_;

        ros::Subscriber local_map_sub_;
        ros::Subscriber livox_scan_sub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber pose_sub_;

        ros::Publisher traj_pub_;
};
