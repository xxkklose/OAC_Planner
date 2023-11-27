#include "oac_mapping.h"

#include "ellipse.hpp"
#include "DBSCAN.hpp"
#include "KM.hpp"



using namespace Eigen;
/*
    author: Xingke Xia email: klosexxk@gmail.com
    用于构建高程栅格地图,将环境信息用于规划
*/

pcl::PassThrough<pcl::PointXYZ> pass_x;
pcl::PassThrough<pcl::PointXYZ> pass_y;
pcl::VoxelGrid<pcl::PointXYZ> sor;

KMAlgorithm KM;

// DBSCAN param
float DBSCAN_R;
int DBSCAN_N;

float step_height;
float obs_height;

int block_size;
int block_num;

Eigen::Vector2d robot_position2d;


Eigen::MatrixXf gradient_map_processing(Eigen::MatrixXf &map_data, vector<DBSCAN::Point> &dataset);

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
    nh_.param("ros_topic/lidar_topic", lidar_sub_topic_, std::string("velodyne_points"));
    
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
    lidar_sub_       = nh_.subscribe
        (lidar_sub_topic_,          1, &Mapping::lidarHandler, this);
    
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map", 1, true);
    local_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("local_map", 1, true);
    lidar_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_cloud", 1, true);

    // param
    nh_.param<float>("localmap_x_size", localmap_x_size, 10);
    nh_.param<float>("localmap_y_size", localmap_y_size, 10);
    nh_.param<float>("resolution", resolution, 0.1);

    nh_.param<float>("obs_height", obs_height, 0.4);
    nh_.param<float>("step_height", step_height, 0.5);
    nh_.param<float>("DBSCAN_R", DBSCAN_R, 5.0);
    nh_.param<int>("DBSCAN_N", DBSCAN_N, 5);

    nh_.param<int>("block_size", block_size, localmap_x_size * _inv_resolution * 0.2);
    nh_.param<int>("block_num", block_num, 5);

    _inv_resolution = 1.0 / resolution;
    map_index_len = localmap_x_size * _inv_resolution;

    sub_map_.setBasicLayers({"elevation", "local_lidar"});
    sub_map_.add("elevation", 0.0);
    sub_map_.add("local_lidar", 0.0);
    // sub_map_.add("gradient_map", 0.0);
    sub_map_.setFrameId("world");
    sub_map_.setGeometry(grid_map::Length(localmap_x_size, localmap_y_size), resolution);

    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-localmap_x_size / 2, localmap_x_size / 2);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-localmap_y_size / 2, localmap_y_size / 2);

    sor.setLeafSize(0.05f, 0.05f, 0.05f);
}

void Mapping::process()
{
    Eigen::MatrixXf lidar_pcd_matrix(map_index_len, map_index_len);
    Eigen::MatrixXf map_interpolate(map_index_len, map_index_len);
    Eigen::MatrixXf gradient_map(map_index_len, map_index_len);

    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        robot_position2d << curr_pos_(0), curr_pos_(1);
        pointCloudTransform(); 
        sub_map_.setPosition(robot_position2d);
        sub_map_.clear("local_lidar");
        lidar_pcd_matrix = sub_map_.get("local_lidar");
        lidar2gridmap(lidar_pcd_matrix);
        sub_map_.add("local_lidar", lidar_pcd_matrix);
        // std::cout<< "lidar_pcd_matrix" << lidar_pcd_matrix << "\n";
        sub_map_.clear("elevation");
        map_interpolate = map_interpolation(lidar_pcd_matrix);
        map_interpolate = map_inflate(map_interpolate);
        sub_map_.add("elevation", map_interpolate);

        // obs map
        vector<DBSCAN::Point> non_clustered_obs;
        gradient_map = gradient_map_processing(map_interpolate, non_clustered_obs);

        // DBSCAN
        DBSCAN DS(DBSCAN_R, DBSCAN_N, non_clustered_obs);
        vector<Obstacle> clustered_obs(DS.cluster_num);
        for (const auto &obs : non_clustered_obs)
        {
            if (obs.obsID > 0)
            {
                gradient_map(obs.x, obs.y) = -0.3;
                clustered_obs[obs.obsID - 1].emplace_back(obs.x, obs.y);
            }
        }

        // // get MSE
        // vector<Ellipse> ellipses_array = get_ellipse_array(clustered_obs, map_);
        // KM.tracking(ellipses_array);
        // ab_variance_calculation(ellipses_array);

        // // publish
        // std_msgs::Float32MultiArray for_obs_track;
        // for (const auto &ellipse : ellipses_array)
        // {
        // if (ellipse.label == 0)
        //     continue;

        // for_obs_track.data.push_back(ellipse.cx);
        // for_obs_track.data.push_back(ellipse.cy);
        // for_obs_track.data.push_back(ellipse.semimajor);
        // for_obs_track.data.push_back(ellipse.semiminor);
        // for_obs_track.data.push_back(ellipse.theta);
        // for_obs_track.data.push_back(ellipse.label);
        // for_obs_track.data.push_back(ellipse.variance);
        // }
        // for_obs_track_pub.publish(for_obs_track);   

        visualMap(sub_map_, map_pub_, "velodyne");
        // visualMap(local_map_, local_map_pub_);
        visualMap(local_map_, local_map_pub_, "camera_init");
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
        filter_chain_.update(sub_map_, sub_map_);
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

    // initStartRegion();
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

void Mapping::visualMap(const grid_map::GridMap& map, const ros::Publisher& map_pub, const std::string& frame_id)
{
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);
    msg.info.header.frame_id = frame_id;
    msg.info.header.stamp = ros::Time::now();
    map_pub.publish(msg);
}

void Mapping::lidarHandler(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg)
{
    pcl::fromROSMsg(*lidar_msg, lidar_cloud_);
    // updateTF();
}

void Mapping::updateTF()
{
    // while(true)
    // {
    //     try
    //     {
    //         tf_listener_.waitForTransform("camera_init", "velodyne", ros::Time(0), ros::Duration(0.1));
    //         tf_listener_.lookupTransform("camera_init", "velodyne", ros::Time(0), lidar_world_transform_);
    //         tf_listener_.waitForTransform("camera_init", "base_link", ros::Time(0), ros::Duration(0.1));
    //         tf_listener_.lookupTransform("camera_init", "base_link", ros::Time(0), base_world_transform_);
    //         break;
    //     }
    //     catch(tf::TransformException& ex)
    //     {
    //         ROS_ERROR("%s", ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }
    // }
}

void Mapping::pointCloudTransform()
{
    // pass_x.setInputCloud(lidar_cloud_.makeShared());
    // pass_x.filter(lidar_cloud_);

    // pass_y.setInputCloud(lidar_cloud_.makeShared());
    // pass_y.filter(lidar_cloud_);

    // lidar_cloud_filter_.clear();
    // sor.setInputCloud(lidar_cloud_.makeShared());
    // sor.filter(lidar_cloud_filter_);

    Eigen::Affine3d affine_transform;
    tf::transformTFToEigen(lidar_world_transform_, affine_transform);
    // pcl::transformPointCloud(lidar_cloud_filter_, lidar_cloud_global_, affine_transform);
    pcl::transformPointCloud(lidar_cloud_, lidar_cloud_global_, Eigen::Affine3f::Identity()); //先用这个代替

    sensor_msgs::PointCloud2 lidar_cloud_global_msg;
    pcl::toROSMsg(lidar_cloud_global_, lidar_cloud_global_msg);
    lidar_cloud_global_msg.header.frame_id = "velodyne";
    lidar_cloud_global_msg.header.stamp = ros::Time::now();
    lidar_cloud_pub_.publish(lidar_cloud_global_msg);
}

void Mapping::lidar2gridmap(Eigen::MatrixXf& lidar_data_matrix)
{
    int col = lidar_data_matrix.cols();
    int row = lidar_data_matrix.rows();
    for(const auto& pt : lidar_cloud_global_)
    {
        int j = (pt.x - robot_position2d(0)) * _inv_resolution + col * 0.5;
        j = min(max(j, 0), row - 1);
        int k = (pt.y - robot_position2d(1)) * _inv_resolution + row * 0.5;
        k = min(max(k, 0), col - 1);

        if(std::isnan(lidar_data_matrix(row - 1 - j, col - 1 - k)))
            lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
        if(lidar_data_matrix(row - 1 - j, col - 1 - k) < pt.z)
            lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
    }
}

Eigen::MatrixXf Mapping::map_interpolation(const Eigen::MatrixXf &map_data)
{
    int col = map_data.cols(), row = map_data.rows();
    Eigen::MatrixXf map_interpolation(map_data);
    for (int i = 1; i < row - 1; i++)
    {
        for (int j = 1; j < col - 1; j++)
        {
        if (isnan(map_data(i, j)))
        {
            int count = 0;
            float height = 0;
            for (int k = 0; k <= 2; k++)
            {
            for (int q = 0; q <= 2; q++)
            {
                if (!isnan(map_data(i - 1 + k, j - 1 + q)))
                {
                count++;
                height += map_data(i - 1 + k, j - 1 + q);
                }
            }
            }
            map_interpolation(i, j) = (count > 0) ? height / count : NAN;
        }
        }
    }
    return map_interpolation;
}

Eigen::MatrixXf Mapping::map_inflate(const Eigen::MatrixXf &map_data)
{
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf map_inflated(map_data);
  for (int i = 3; i < row - 3; i++)
  {
    for (int j = 3; j < col - 3; j++)
    {
      if (isnan(map_data(i, j)))
        continue;

      double dis = sqrt((i - col / 2) * (i - col / 2) + (j - col / 2) * (j - col / 2));
      int radius;
      if (dis < col / 3)
        radius = 1;
      else if (dis < col * 0.45)
        radius = 2;
      else
        radius = 3;
      map_inflate_block(map_inflated, map_data, i, j, radius);
    }
  }
  return map_inflated;
}

void Mapping::map_inflate_block(Eigen::MatrixXf &dst, const Eigen::MatrixXf &src, int startRow, int startCol, int radius)
{
  for (int k = 0; k <= 2 * radius; k++)
  {
    for (int q = 0; q <= 2 * radius; q++)
    {
      if (isnan(src(startRow - radius + k, startCol - radius + q)))
      {
        dst(startRow - radius + k, startCol - radius + q) = src(startRow, startCol);
      }
    }
  }
}

Eigen::MatrixXf gradient_map_processing(Eigen::MatrixXf &map_data, vector<DBSCAN::Point> &dataset)
{
  const float threshold = -1.25;
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf gradient_map(row, col);
  gradient_map.setOnes();
  gradient_map *= threshold;
  DBSCAN::Point obs;

  for (int i = 1; i < row - 1; i++)
  {
    for (int j = 1; j < col - 1; j++)
    {
      bool has_nan_value = false;
      for (int p = -1; p <= 1; p++)
      {
        for (int q = -1; q <= 1; q++)
        {
          if (isnan(map_data(i + p, j + q)))
          {
            gradient_map(i, j) = threshold;
            has_nan_value = true;
          }
        }
      }
      if (!has_nan_value)
      {
        float sobel_x = map_data(i + 1, j + 1) - map_data(i - 1, j + 1) + map_data(i + 1, j) - map_data(i - 1, j) +
                        map_data(i + 1, j - 1) - map_data(i - 1, j - 1);
        float sobel_y = map_data(i - 1, j + 1) - map_data(i - 1, j - 1) + map_data(i, j + 1) - map_data(i, j - 1) +
                        map_data(i + 1, j + 1) - map_data(i + 1, j - 1);
        gradient_map(i, j) = sqrt(sobel_x * sobel_x + sobel_y * sobel_y) + threshold;
        if (gradient_map(i, j) > obs_height + threshold)
        {
          obs.x = i;
          obs.y = j;
          dataset.push_back(obs);
        }
        else
        {
          Eigen::MatrixXf::Index minRow, minCol;
          Eigen::MatrixXf block = map_data.block(i / block_size, j / block_size, block_size, block_size);
          float min = block.minCoeff(&minRow, &minCol);
          if (map_data(i, j) - min > step_height)
          {
            obs.x = i;
            obs.y = j;
            dataset.push_back(obs);
          }
        }
      }
    }
  }

  return gradient_map;
}