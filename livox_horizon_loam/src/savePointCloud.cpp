#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/filters/passthrough.h>

pcl::PointCloud<pcl::PointXYZ> totalCloud;
ros::Publisher pub_total; 


void originCallBack(const sensor_msgs::PointCloud2ConstPtr& msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_pointcloud);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (true && ros::ok())
    {
      try
      {
        listener.lookupTransform("camera_init", "aft_mapped", ros::Time(0), transform);  //查询变换
        break;
      }
      catch (tf::TransformException& ex)
      {
        continue;
      }
    }
    ROS_WARN("here");
    std::cout<< transform.getOrigin().x() <<std::endl;
    std::cout<< transform.getOrigin().y() <<std::endl;
    std::cout<< transform.getOrigin().z() <<std::endl;
    double z_threshold = transform.getOrigin().z();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_pointcloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-10000000000000000, z_threshold + 2);  // 过滤掉 Z 坐标大于 z_threshold 的点
    pass.filter(*filtered_cloud);

    totalCloud += *filtered_cloud;
    // 创建一个PointCloud2消息
    sensor_msgs::PointCloud2 cloud_msg;

    // 将pcl::PointCloud<pcl::PointXYZ>转换为sensor_msgs::PointCloud2
    pcl::toROSMsg(totalCloud, cloud_msg);
    // 填充PointCloud2消息的头部信息（frame_id、timestamp等）

    cloud_msg.header.frame_id = "world"; // 设置坐标系
    cloud_msg.header.stamp = ros::Time::now(); // 设置时间戳

    // 发布PointCloud2消息
    pub_total.publish(cloud_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "savePointCloud");
    ros::NodeHandle nh;
    
    ros::Subscriber sub_origin = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100, originCallBack);

    pub_total = nh.advertise<sensor_msgs::PointCloud2>("livox_total_point", 1);

    ros::spin();
    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}