#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_filters/BufferNormalizerFilter.hpp>
#include <grid_map_filters/NormalVectorsFilter.hpp>
#include <filters/filter_chain.h>
#include <execution>
#include <chrono>
#include <thread>
#include <algorithm>

class Mapping
{

public:
    Mapping();
    ~Mapping();

    void init();
    void process();
    void exit();

    void odomHandler(const nav_msgs::Path::ConstPtr& odom_msg);
    void pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

    void updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud);
    void initStartRegion();
    bool localMapProcess(const grid_map::Position& local_map_center, const grid_map::Length& local_map_length);
    void visualMap(const grid_map::GridMap& map, const ros::Publisher& map_pub);

private:
    // ros related
    ros::NodeHandle nh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber point_cloud_sub_;

    ros::Publisher map_pub_;
    ros::Publisher local_map_pub_;

    std::string odom_sub_topic_;
    std::string point_cloud_sub_topic_;
    bool has_map_ = false;

    pcl::PassThrough<pcl::PointXYZ> pass_;


    Eigen::Vector3d curr_pos_ = {0, 0, 0};

    grid_map::GridMap map_;
    grid_map::GridMap local_map_;
    filters::FilterChain<grid_map::GridMap> filter_chain_{"grid_map::GridMap"};
};


