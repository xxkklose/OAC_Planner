#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "livox_ros_driver/CustomMsg.h"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
ros::Publisher pub_pcl;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
void multi_callback(const livox_ros_driver::CustomMsgConstPtr& livox_msg_1,
                    const livox_ros_driver::CustomMsgConstPtr& livox_msg_2) {
  auto t1 = std::chrono::steady_clock::now();
  auto timediff = livox_msg_1->timebase - livox_msg_2->timebase;

  livox_ros_driver::CustomMsg livox_msg;
  livox_msg.header = livox_msg_1->header;
  livox_msg.header.frame_id = "camera_init";
  livox_msg.timebase = livox_msg_1->timebase;
  livox_msg.point_num = livox_msg_1->point_num + livox_msg_2->point_num;
  // livox_msg.point_num = livox_msg_1->point_num;
  // livox_msg.point_num = livox_msg_2->point_num;
  // livox_msg.points.resize(livox_msg.point_num);
  livox_msg.lidar_id = livox_msg_1->lidar_id;
  livox_msg.rsvd = livox_msg_1->rsvd;

  for (size_t i = 0; i < livox_msg_2->point_num; i++)
  {
    auto pt = livox_msg_2->points[i];
    pt.offset_time += timediff;
    livox_msg.points.push_back(pt); 
  }

  for (size_t i = 0; i < livox_msg_1->point_num; i++)
  {
    livox_msg.points.push_back(livox_msg_1->points[i]);
  }

  pub_pcl.publish(livox_msg);
  auto t2 = std::chrono::steady_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
  // std::cout << "pointcloud time consume: " << time.count() << "\n";
  // time consume 20ms 

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  // ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
  //     "/livox/lidar", 100, LivoxMsgCbk1);
  
  message_filters::Subscriber<livox_ros_driver::CustomMsg> sub_horizon (nh, "/livox/lidar1", 1000, ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<livox_ros_driver::CustomMsg> sub_mid360 (nh, "/livox/lidar2", 1000, ros::TransportHints().tcpNoDelay());

  typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver::CustomMsg, livox_ros_driver::CustomMsg> syncPolicy;
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_horizon, sub_mid360);  
  sync.registerCallback(boost::bind(&multi_callback, _1, _2));

  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 100);
  pub_pcl = nh.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 100);

  ros::spin();
}