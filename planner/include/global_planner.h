#pragma once

#include "backward.hpp"
#include "planner.h"
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

using namespace std;
using namespace std_msgs;
using namespace Eigen;
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

        // ros related 激光点云地图、中间点、位姿、返回模式、对齐模式
        ros::Subscriber map_sub_, wp_sub_, pose_sub_, returnMode_sub_, alignMode_sub_;

        ros::Publisher grid_map_vis_pub_;
        ros::Publisher path_vis_pub_;
        ros::Publisher goal_vis_pub_;
        ros::Publisher surf_vis_pub_;
        ros::Publisher tree_vis_pub_;
        ros::Publisher path_interpolation_pub_;
        ros::Publisher tree_tra_pub_;
        ros::Publisher pose_pub_to_control_;
        ros::Publisher traj_jerk_vis_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher path_to_control_;
        ros::Publisher cloud_registered_pub_;
        ros::Publisher marker_pub_box_;

        // 运动模式
        enum MotionState
        {
            SearchMode,
            ReturnMode,
            AlignMode
        };

        MotionState motionState_ = SearchMode;

        std::mutex dataMutex_;

        pcl::PointCloud<PointT>::Ptr keyPoints_; //TODO: 添加初始化函数进行处理
        pcl::KdTreeFLANN<PointT> kdtree_;
        int K = 5;
        Vector3d lastKeyPoint_;
        Vector3d normalAtObs_;
        Vector3d pointAtObs_;

        // 规划参数，从map1.yaml中读取
        double resolution_;
        double goal_thre_;
        double step_size_;
        double h_surf_car_;
        double max_initial_time_;
        double radius_fit_plane_;
        FitPlaneArg fit_plane_arg_;
        double neighbor_radius_;

        //动态保存点云地图
        std::queue<pcl::PointCloud<pcl::PointXYZ>> pointcloud_map_queue_;
        int queue_size_ = 20;
        std::queue<std::pair<PointCloud, Vector3d>> pointcloud_vector3d_queue_;
        int queue2_size_ = 15; //如果卡顿修改他
        Vector3d last_point_ = {0,0,0};
        pcl::PassThrough<pcl::PointXYZ> pass_; //直通滤波过滤器
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_; //StatisticalOutlierRemoval过滤器
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_; //半径滤波过滤器

        // useful global variables
        Vector3d start_pt_ = {0,0,0};
        Vector3d target_pt_;
        geometry_msgs::PoseStamped start_pose_ = geometry_msgs::PoseStamped();
        World* world_ = NULL;
        Minimum_jerk mj_ = Minimum_jerk();
        PFRRTStar* pf_rrt_star_ = NULL;
        double max_vel_;
        double max_acc_;

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
        void pointCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_registered_msg);
        void multi_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_registered_msg);
        void rcvWaypointsCallback(const nav_msgs::Path& wp);
        void rcvPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &surfmap_msg);
        void rcvPoseCallback(const geometry_msgs::PoseStamped& pose);
        void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub);
        void findSolution();
        void callPlanner();
        void pubPathToControl(ros::Publisher* path_to_control_pub);
        void motionModeDetect();
        void returnModeCallback(const std_msgs::String& msg);
        void alignModeCallback(const std_msgs::String& msg);
        void visualBox(const geometry_msgs::PoseStamped& pose);
        void exit();
};
    

    