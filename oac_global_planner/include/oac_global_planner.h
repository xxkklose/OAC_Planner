#pragma once

#include "backward.hpp"
#include "pf_rrt_planner.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
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
#include <ros/package.h>

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace grid_map;
using namespace OAC;
using namespace OAC::visualization;
using namespace OAC::planner;

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;


class GlobalPlanner  //全局规划类
{
    private:
        std::ofstream outputFile_;
        std::ofstream keyPointDebug_;
        std::ofstream alignPointDebug_;
        std::ofstream treeDebug_;
        std::ofstream logFile_;

        // ros related 激光点云地图、中间点、位姿、返回模式、对齐模式
        ros::Subscriber map_sub_, local_map_sub_, wp_sub_, pose_sub_, returnMode_sub_, alignMode_sub_;
        
        std::string pose_sub_topic_;
        std::string map_sub_topic_;
        std::string local_map_sub_topic_;

        ros::Publisher grid_map_vis_pub_;
        ros::Publisher path_vis_pub_;
        ros::Publisher goal_vis_pub_;
        ros::Publisher surf_vis_pub_;
        ros::Publisher tree_vis_pub_;
        ros::Publisher tree_tra_pub_;
        ros::Publisher global_path_pub_;
        ros::Publisher marker_pub_box_;

        ros::Publisher a_star_path_pub_;

        // 运动模式
        enum MotionState
        {
            SearchMode,
            ReturnMode,
            AlignMode
        };

        MotionState motionState_ = SearchMode;

        std::mutex gp_mutex_;

        pcl::PointCloud<PointT>::Ptr keyPoints_; 
        pcl::KdTreeFLANN<PointT> kdtree_;
        int K = 5;
        Vector3d lastKeyPoint_;
        Vector3d normalAtObs_;
        Vector3d pointAtObs_;

        // 规划参数，从param.yaml中读取
        double resolution_;
        double goal_thre_;
        double step_size_;
        double h_surf_car_;
        double max_initial_time_;
        double radius_fit_plane_;
        FitPlaneArg fit_plane_arg_;
        double neighbor_radius_;

        //动态保存点云地图
        Vector3d last_point_ = {0,0,0};
        pcl::PassThrough<pcl::PointXYZ> pass_; //直通滤波过滤器

        // useful global variables
        Vector3d start_pt_ = {0,0,0};
        Vector3d planning_start_pt_ = {0,0,0};
        Vector3d target_pt_;
        geometry_msgs::PoseStamped start_pose_ = geometry_msgs::PoseStamped();
        World* world_ = NULL;
        PFRRTStar* pf_rrt_star_ = NULL;
        Path compare_path_ = Path();
        Path solution_ = Path();
        int solution_not_change_count_ = 0;

        // indicate whether the robot has a moving goal
        // 根据该标志设置是否由目标点
        bool has_goal_ = false;

        double plane_bottom_;
        double planning_horizon_;
        double planning_time_horizon_; 

    public:
        GlobalPlanner(/* args */);
        ~GlobalPlanner();

        void init(ros::NodeHandle& nh);
        void process();
        void mapCallback(const grid_map_msgs::GridMap& map_msg);
        void localMapCallback(const grid_map_msgs::GridMap& map_msg);
        void rcvWaypointsCallback(const nav_msgs::Path& wp);
        void rcvPoseCallback(const nav_msgs::Path& pose);
        void removeGlobalPathPoints(Path& solution);
        bool needChangeGlobalPath(const Path &solution);
        bool checkSolutionCollision();
        void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub);
        void findSolution();
        void callPlanner();
        void callAStar();
        void pubPathToControl(ros::Publisher* path_to_control_pub);
        void motionModeDetect();
        void returnModeCallback(const std_msgs::String& msg);
        void visualBox(const geometry_msgs::PoseStamped& pose);
        void plotLog();
        void exit();

        bool run_time_log_ = false;
        bool run_time_print_ = false;

        // log struct
        struct logPlot
        {
            // 主要模块耗时
            double main_loop_time;
            double map_construction_time;
            double planning_time;
            double vis_time;
            bool updated;

            // 回调函数耗时
            double multi_callback_time;
            double rcv_waypoints_callback_time;
            double rcv_pose_callback_time;
            double return_mode_callback_time;
            double align_mode_callback_time;
        };

        logPlot log_data_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
    

    