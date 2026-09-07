#ifndef QIQI2026_CHASE__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_
#define QIQI2026_CHASE__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp" 
#include "base_interfaces_demo/msg/target.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"


using PointStamped = geometry_msgs::msg::PointStamped;
using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;

namespace qiqi2026_chase
{

class CalculateAttackPose : public rclcpp::Node
{
public:
  CalculateAttackPose();//构造函数
  bool chase();



private:
  struct Parameters
  {
    double attack_radius;//攻击半径
    int num_sectors;//扇区数量
    int cost_threshold;//成本阈值
    std::string costmap_topic;//代价地图话题
    std::string robot_base_frame;//机器人基础框架
    double transform_tolerance;//变换公差
    double max_visualization_distance;//最大可视化距离
    double marker_scale_base;//标记基础缩放
    bool visualize;//可视化
  };


  std::vector<Point> generateCandidatePoints(const Point & enemy_position);//产生候选点

  std::vector<Point> filterFeasiblePoints(
    const std::vector<Point> & candidates, const nav_msgs::msg::OccupancyGrid & costmap);//过滤可行点

  Point selectBestPoint(const std::vector<Point> & feasible_points, const Point & robot_position);//选择最佳点

  //PoseStamped createAttackPose(const Point & attack_point, const PointStamped & enemy_position);//创建攻击位姿

  void createVisualizationMarkers(//创建可视化标记
    visualization_msgs::msg::MarkerArray & msg, const Point & enemy_position,
    const std::vector<Point> & candidates, const std::vector<Point> & feasible_points,
    const Point & robot_position, const nav_msgs::msg::OccupancyGrid & costmap);

  bool transformPoseInTargetFrame(//在目标框架中变换位姿
    const PointStamped & input_pose, PointStamped & transformed_pose, tf2_ros::Buffer & tf_buffer,
    const std::string target_frame, const double transform_timeout);//变换位姿在目标框架中

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Parameters params_;

  PointStamped enemy_on_costmap_;//敌人在成本地图上的位置
    




  rclcpp::TimerBase::SharedPtr timer_;


  //获得目标信息
  rclcpp::Subscription<base_interfaces_demo::msg::Target>::SharedPtr target_sub_;//订阅/target话题
  void targetCallback(base_interfaces_demo::msg::Target::SharedPtr msg);
  std_msgs::msg::Header tracker_head;//消息头
  bool tracking = false;
  //bool target_received_ = false;
  float tracker_dis = 0.0f;
  float tracker_yaw = 0.0f;
  
  //获得全局代价地图信息
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;//订阅/costmap话题
  void szxnb(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;//全局代价地图

  //发布追击点
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr chase_point_pub_;//发布chase_point话题
  geometry_msgs::msg::Point chase_point_;//追击点

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;//发布可视化标记


};

}  // namespace qiqi2026_chase

#endif  // QIQI2026_CHASE__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_
