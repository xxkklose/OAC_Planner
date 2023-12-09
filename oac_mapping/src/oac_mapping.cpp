#include "oac_mapping.h"
#include "segment.hpp"  //提取地面

#include "ellipse.hpp"
#include "DBSCAN.hpp"
#include "KM.hpp"



using namespace Eigen;
/*
    author: Xingke Xia email: klosexxk@gmail.com
    用于构建高程栅格地图,将环境信息用于规划
*/

Mapping::Mapping()
    : nh_("~")
{
}

Mapping::~Mapping()
{
}

void Mapping::init()
{
    nh_.param("ros_topic/odom_topic", odom_sub_topic_, std::string("odom"));
    nh_.param("ros_topic/lidar_topic", point_cloud_sub_topic_, std::string("point_cloud"));
    nh_.param("ros_topic/lidar_topic", lidar_sub_topic_, std::string("velodyne_points"));
    nh_.param("plane_bottom", default_height_, -0.37);
    
    // map_.setBasicLayers({"elevation"});
    map_.setFrameId("camera_init");
    map_.setGeometry(grid_map::Length(100, 100), 0.08);
    map_.add("elevation", default_height_);
    lidar_map_.setFrameId("camera_init");
    lidar_map_.setGeometry(grid_map::Length(100, 100), 0.08);
    lidar_map_.add("elevation", default_height_);


    if(filter_chain_.configure("grid_map_filters", nh_))
    {
        ROS_INFO("Successfully configured the GridMap filter chain.");
    }
    else
    {
        ROS_ERROR("Failed to configure the GridMap filter chain.");
    }

    odom_sub_        = nh_.subscribe
        (odom_sub_topic_,           1, &Mapping::odomHandler, this);
    point_cloud_sub_ = nh_.subscribe
        (point_cloud_sub_topic_,    1, &Mapping::pointCloudHandler, this);
    lidar_sub_       = nh_.subscribe
        (lidar_sub_topic_,          1, &Mapping::lidarHandler, this);
    
    map_pub_            = nh_.advertise<grid_map_msgs::GridMap>
        ("map",                     1, true);
    local_map_pub_      = nh_.advertise<grid_map_msgs::GridMap>
        ("local_map",               1, true);
    lidar_cloud_pub_    = nh_.advertise<grid_map_msgs::GridMap>
        ("lidar_map",               1, true);
    visual_map_pub_     = nh_.advertise<grid_map_msgs::GridMap>
        ("visual_map",              1, true);
    visual_local_map_pub_    = nh_.advertise<grid_map_msgs::GridMap>
        ("visual_local_map",        1, true);


}

void Mapping::process()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        if(has_map_)
        {        
            visualMap(map_, map_pub_, "camera_init");
            visualMap(local_map_, local_map_pub_, "camera_init");
            visualMap(lidar_map_, lidar_cloud_pub_, "camera_init");
            visualMap(visual_map_, visual_map_pub_, "camera_init");
            visualMap(visual_local_map_, visual_local_map_pub_, "camera_init");
        }
        ros::spinOnce();
        rate.sleep();
    }
    if(!ros::ok())
    {
        exit();
    }
}

void Mapping::exit()
{
    ROS_INFO("Exiting...");
}

void Mapping::pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto start_time = std::chrono::steady_clock::now();
    pcl::fromROSMsg(*point_cloud_msg, *cloud);

    tf::StampedTransform transform;
    try{
        tf_listener_.waitForTransform("camera_init", "velodyne", ros::Time(0), ros::Duration(0.1));
        tf_listener_.lookupTransform("camera_init", "velodyne", ros::Time(0), transform);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform, transform_eigen);
    pcl::transformPointCloud(*cloud, *cloud, transform_eigen.matrix());

    pass_.setInputCloud(cloud);
    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(-9999, curr_pos_(2) + 1.5);
    pass_.filter(*cloud);

    updateLidarMap(cloud);
    updateMap(cloud);
}

void Mapping::lidarHandler(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // auto start_time = std::chrono::steady_clock::now();
    // pcl::fromROSMsg(*lidar_msg, *cloud);
    
    // tf::StampedTransform transform;
    // try{
    //     tf_listener_.waitForTransform("camera_init", "velodyne", ros::Time(0), ros::Duration(0.1));
    //     tf_listener_.lookupTransform("camera_init", "velodyne", ros::Time(0), transform);
    // }
    // catch(tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    // }
    // Eigen::Affine3d transform_eigen;
    // tf::transformTFToEigen(transform, transform_eigen);
    // pcl::transformPointCloud(*cloud, *cloud, transform_eigen.matrix());

    // pass_.setInputCloud(cloud);
    // pass_.setFilterFieldName("z");
    // pass_.setFilterLimits(-9999, curr_pos_(2) + 2);
    // pass_.filter(*cloud);
}

void Mapping::updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud)
{
    if(has_lidar_map_)
    {
        std::for_each(std::execution::par, point_cloud->begin(), point_cloud->end(), [&](const auto& pt) {  
            Vector3d obstacle(pt.x, pt.y, pt.z);  
            if(!(abs(pt.x - curr_pos_.x()) < 0.4 && abs(pt.y - curr_pos_.y()) < 0.4 && abs(pt.z - curr_pos_.z()) < 0.6))
            {
                grid_map::Position position(obstacle(0), obstacle(1));
                if(map_.isInside(position) && lidar_map_.isInside(position))
                {
                    if(abs(lidar_map_.atPosition("elevation", position) - obstacle(2)) < 1e-6 )
                        map_.atPosition("elevation", position) = obstacle(2);
                    else{
                        map_.atPosition("elevation", position) = lidar_map_.atPosition("elevation", position);
                    }
                }
            }
        });  
        has_map_ = true;
        has_lidar_map_ = false;
    }
}

void Mapping::updateLidarMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud)
{
    lidar_map_.erase("elevation");
    lidar_map_.add("elevation", default_height_);
    std::for_each(std::execution::par, point_cloud->begin(), point_cloud->end(), [&](const auto& pt) {  
        Vector3d obstacle(pt.x, pt.y, pt.z);  
        if(!(abs(pt.x - curr_pos_.x()) < 0.6 && abs(pt.y - curr_pos_.y()) < 0.6 && abs(pt.z - curr_pos_.z()) < 0.65))
        {
            grid_map::Position position(obstacle(0), obstacle(1));
            if(lidar_map_.isInside(position)) lidar_map_.atPosition("elevation", position) = obstacle(2);
        }
    });  
    has_lidar_map_ = true;
} 

void Mapping::odomHandler(const nav_msgs::Path::ConstPtr& odom_msg)
{
    curr_pos_ << odom_msg->poses.back().pose.position.x,
                 odom_msg->poses.back().pose.position.y,
                 odom_msg->poses.back().pose.position.z;

    grid_map::Position local_map_center(curr_pos_(0), curr_pos_(1));
    grid_map::Length local_map_length(20, 20);

    bool get_local_map = localMapProcess(local_map_center, local_map_length);
    
}

bool Mapping::localMapProcess(const grid_map::Position& local_map_center, const grid_map::Length& local_map_length)
{
    bool flag = false;
    local_map_ = map_.getSubmap(local_map_center, local_map_length, flag);
    grid_map::Length visual_map_length(20, 20);
    visual_map_ = lidar_map_.getSubmap(local_map_center, visual_map_length, flag);
    grid_map::Length visual_local_map_length(10, 10);
    visual_local_map_ = lidar_map_.getSubmap(local_map_center, visual_local_map_length, flag);
    if(flag)
    {
        local_map_.add("collision", 0.0);
        local_map_.add("dilatation_barrier", 0.0);
        visual_map_.add("collision", 0.0);
        visual_map_.add("dilatation_barrier", 0.0);
        for(grid_map::GridMapIterator it(local_map_); !it.isPastEnd(); ++it)
        {
            if(local_map_.at("elevation", *it) > curr_pos_(2) - 0.6)
            {
                local_map_.at("collision", *it) = 1.0;
            }
        }
        for(grid_map::GridMapIterator it(visual_map_); !it.isPastEnd(); ++it)
        {
            if(visual_map_.at("elevation", *it) > curr_pos_(2) - 0.6)
            {
                visual_map_.at("collision", *it) = 1.0;
            }
        }
        for(grid_map::GridMapIterator it(local_map_); !it.isPastEnd(); ++it){
            if(local_map_.at("collision", *it) == 1.0){
                // 把周围半径为0.35m的区域的dilatation_barrier设置为1.0;
                grid_map::Position position;
                local_map_.getPosition(*it, position);
                for(grid_map::CircleIterator it_circle(local_map_, position, 0.4); !it_circle.isPastEnd(); ++it_circle)
                {
                    local_map_.at("dilatation_barrier", *it_circle) = 1.0;
                }
            } 
        } 
        for(grid_map::GridMapIterator it(visual_map_); !it.isPastEnd(); ++it){
            if(visual_map_.at("collision", *it) == 1.0){
                // 把周围半径为0.3m的区域的dilatation_barrier设置为1.0;
                grid_map::Position position;
                visual_map_.getPosition(*it, position);
                for(grid_map::CircleIterator it_circle(visual_map_, position, 0.2); !it_circle.isPastEnd(); ++it_circle)
                {
                    visual_map_.at("dilatation_barrier", *it_circle) = 1.0;
                }
            } 
        } 
        filter_chain_.update(local_map_, local_map_);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get local map!");
        return false;
    }
}

void Mapping::visualMap(const grid_map::GridMap& map, const ros::Publisher& map_pub, const std::string& frame_id)
{
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);
    msg.info.header.frame_id = frame_id;
    msg.info.header.stamp = ros::Time::now();
    map_pub.publish(msg);
}
