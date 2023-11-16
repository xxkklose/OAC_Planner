#pragma once

// #include "backward.hpp"
// #include "planner_classes.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "livox_ros_driver/CustomMsg.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
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

                State(const double x, const double y, const double z, const double yaw, const double velocity, const double yaw_rate){
                    x_ = x;
                    y_ = y;
                    z_ = z;
                    yaw_ = yaw;
                    velocity_ = velocity;
                    yaw_rate_ = yaw_rate;
                };

                double x_;
                double y_;
                double z_;
                double yaw_;
                double velocity_;
                double yaw_rate_;
        };

        class Window
        {
            public:
                Window(void) : min_v_(0.0), max_v_(0.0), min_yaw_rate_(0.0), max_yaw_rate_(0.0){};
                ~Window(){};

                Window(const double min_v, const double max_v, const double min_yaw_rate, const double max_yaw_rate){
                    min_v_ = min_v;
                    max_v_ = max_v;
                    min_yaw_rate_ = min_yaw_rate;
                    max_yaw_rate_ = max_yaw_rate;
                };

                double min_v_;
                double max_v_;
                double min_yaw_rate_;
                double max_yaw_rate_;
        };

        class TrajectoryCost
        {
            public:
                TrajectoryCost(void) : 
                    heading_cost_(0.0), 
                    velocity_cost_(0.0),
                    clearance_cost_(0.0), 
                    dist_cost_(0.0), 
                    traj_cost_(0.0), 
                    velocity_(0.0), 
                    yaw_rate_(0.0){};
                ~TrajectoryCost(){};

                TrajectoryCost(const double heading_cost, const double velocity_cost, const double clearance_cost, const double dist_cost, const double traj_cost, const double velocity, const double yaw_rate){
                    heading_cost_ = heading_cost;
                    velocity_cost_ = velocity_cost;
                    clearance_cost_ = clearance_cost;
                    dist_cost_ = dist_cost;
                    traj_cost_ = traj_cost;
                    velocity_ = velocity;
                    yaw_rate_ = yaw_rate;
                };

                double heading_cost_;
                double velocity_cost_;
                double clearance_cost_;
                double dist_cost_;
                double traj_cost_;

                double velocity_;
                double yaw_rate_;
        };

        // 局部规划类参数初始化
        void init();
        void process();
        void localMapHandler(const grid_map_msgs::GridMap::ConstPtr& msg);
        void scanHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
        void livoxScanHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg);
        void globalPathHandler(const nav_msgs::Path::ConstPtr& msg);
        void poseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void poseHandler2(const nav_msgs::Path::ConstPtr& msg);
        void goalHandler(const geometry_msgs::PoseStamped::ConstPtr& msg);

        //
        bool inside_box(const pcl::PointXYZ& pt);

        // 判断当前位置在全局路径中的位置
        void check_index();
        double cal_point_angle(const Eigen::Vector3d &s_pt, const Eigen::Vector3d &t_pt);
        // 判断是否到达目标点
        bool has_reached();
        // 到达目标点后重置
        void reset();

        std::vector<double> DynamicWindowApproach();
        Window calc_dynamic_window();
        // 生成轨迹
        std::vector<State> \
        generate_trajectory(const double& velocity, const double& yaw_rate);
        std::vector<State> \
        generate_trajectory(const double& yaw_rate); 
        // 计算最短距离
        double cal_clearance(const std::vector<State>& trajectory);
        // 计算停止距离
        double cal_breaking_dist(const double& velocity);
 

        void can_move();
        bool can_adjust_robot_direction();
        void pub_cmd_vel(const std::vector<double>& param);

       // 按照给定速度和角速度运动
        void motion(State& state, const double velocity, const double yaw_rate);
        // 碰撞检测
        bool check_collision(const std::vector<State>& trajectory);

        void visual();

        // 贝塞尔曲线
        double bernstein(const int& i, const int& n, const double& t);
        Eigen::Vector3d bezier_curve(const std::vector<Eigen::Vector3d>& control_points, const double& t);
        Eigen::Vector3d bezier_curve_derivative(const std::vector<Eigen::Vector3d>& control_points, const double& t);
        std::vector<double> BezierCurveApproach();

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
        // 计算
        // 计算3维距离
        inline double cal_3D_dis(const Eigen::Vector3d& s_pt, const Eigen::Vector3d& t_pt)
        {return (s_pt - t_pt).norm();}
        Eigen::Matrix3d computeRotationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
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

        double h_surf_car_; //机器人高度
        State robot_state_; // 机器人状态
        nav_msgs::Path global_path_; // 全局路径
        geometry_msgs::PoseStamped curr_pose_; // 当前位置
        bool get_global_path_ = false; // 是否获取到全局路径
        Eigen::Vector3d next_target_;
        double curr_t_ = 0.0;
        Eigen::Vector3d goal_; // 目标点
        Eigen::Quaterniond goal_quat_; // 目标点四元数
        geometry_msgs::PoseStamped goal_rel_; // 目标点相对当前位置位姿
        tf::TransformListener tf_listener_; // tf监听器
        std::vector<Eigen::Vector3d> bezier_curve_points_; //贝塞尔曲线点
        std::vector<Eigen::Vector3d> bezier_curve_derivative_; //贝塞尔曲线速度

        double dist_to_goal_th_; // 到达目标点距离阈值
        double angle_to_goal_th_; // 到达目标点角度阈值

        // 条件标志
        bool use_livox_scan_as_input_; // 使用livox 扫描帧作为输入
        bool use_local_map_as_input_; // 使用grid_map局部地图作为输入
        bool use_scan_as_input_; // 使用激光雷达扫描帧作为输入
        bool has_reached_ = false; // 是否到达目标点
        bool has_goal_ = false; // 是否有目标点
        bool has_local_map_;
        bool has_test_ =false;

        // 局部地图障碍
        grid_map::GridMap local_map_;
        geometry_msgs::PoseArray obs_list_;
        std::mutex mutex_;

        // ros相关
        ros::NodeHandle nh_;

        ros::Subscriber local_map_sub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber livox_scan_sub_;
        ros::Subscriber global_path_sub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber odom_sub_;

        ros::Publisher traj_pub_;
        ros::Publisher best_traj_pub_;
        ros::Publisher obs_pub_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher bezier_curve_pub_;
        ros::Publisher next_target_pub_;

        std::string local_map_sub_topic_;
        std::string global_path_sub_topic_;
        std::string odom_sub_topic_;

        // 可视化
        std::vector<std::vector<State>> traj_list_;
        std::vector<State> best_traj_;
        sensor_msgs::PointCloud2 ob_cloud_;
};
