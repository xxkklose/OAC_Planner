#include "oac_mapping.h"
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
    nh_.param("ros_topic/point_cloud_topic", point_cloud_sub_topic_, std::string("point_cloud"));
    
    map_.setBasicLayers({"elevation"});
    map_.add("elevation", 0.0);
    map_.setFrameId("world");
    map_.setGeometry(grid_map::Length(100, 100), 0.08);

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
    
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map", 1, true);
    local_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("local_map", 1, true);


}

void Mapping::process()
{
    // std::chrono::milliseconds interval(100);
    // std::thread map_timer([&]() {
    //     while(ros::ok()){
    //         // std::this_thread::sleep_for(interval);

    //         if(has_map_){
    //             visualMap(map_, map_pub_);
    //             visualMap(local_map_, local_map_pub_);
    //         }
    //     }
    // });   

    ros::Rate rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        visualMap(map_, map_pub_);
        visualMap(local_map_, local_map_pub_);
        rate.sleep();
    }
    if(!ros::ok())
    {
        exit();
    }

    // map_timer.join();
}

void Mapping::exit()
{
    ROS_INFO("Exiting...");
}

void Mapping::odomHandler(const nav_msgs::Path::ConstPtr& odom_msg)
{
    curr_pos_ << odom_msg->poses.back().pose.position.x,
                 odom_msg->poses.back().pose.position.y,
                 odom_msg->poses.back().pose.position.z;

    grid_map::Position local_map_center(curr_pos_(0), curr_pos_(1));
    grid_map::Length local_map_length(10, 10);

    bool get_local_map = localMapProcess(local_map_center, local_map_length);
    
}

bool Mapping::localMapProcess(const grid_map::Position& local_map_center, const grid_map::Length& local_map_length)
{
    bool flag = false;
    local_map_ = map_.getSubmap(local_map_center, local_map_length, flag);
    if(flag)
    {
        local_map_.add("collision", 0.0);
        for(grid_map::GridMapIterator it(local_map_); !it.isPastEnd(); ++it)
        {
            if(local_map_.at("elevation", *it) > curr_pos_(2) - 0.2)
            {
                local_map_.at("collision", *it) = 1.0;
            }
            else
                local_map_.at("collision", *it) = 0.0;
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

void Mapping::pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto start_time = std::chrono::steady_clock::now();
    pcl::fromROSMsg(*point_cloud_msg, *cloud);
    

    pass_.setInputCloud(cloud);
    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(-9999, curr_pos_(2) + 2);
    pass_.filter(*cloud);

    updateMap(cloud);

    initStartRegion();
}

void Mapping::updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud)
{
    std::for_each(std::execution::par, point_cloud->begin(), point_cloud->end(), [&](const auto& pt) {  
        Vector3d obstacle(pt.x, pt.y, pt.z);  
        if(!(abs(pt.x - curr_pos_.x()) < 0.4 && abs(pt.y - curr_pos_.y()) < 0.4 && abs(pt.z - curr_pos_.z()) < 0.6))
        {
            grid_map::Position position(obstacle(0), obstacle(1));
            map_.atPosition("elevation", position) = obstacle(2);
        }
    });  
    has_map_ = true;
}

void Mapping::initStartRegion()
{
    for(int i=-5; i<=5; i++)
    {
        for(int j=-5; j<=5; j++)
        {
            grid_map::Position pos(i*0.1,j*0.1);
            map_.atPosition("elevation", pos) = -0.37;
        }
    }
}

void Mapping::visualMap(const grid_map::GridMap& map, const ros::Publisher& map_pub)
{
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);
    msg.info.header.frame_id = "camera_init";
    msg.info.header.stamp = ros::Time::now();
    map_pub.publish(msg);
}