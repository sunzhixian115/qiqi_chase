#include "calculate_attack_pose.hpp"
#include "base_interfaces_demo/msg/target.hpp"
#include <algorithm>
#include <cmath>
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"


using nav2_util::declare_parameter_if_not_declared;
namespace qiqi2026_chase
{

CalculateAttackPose::CalculateAttackPose(): Node("chase")
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  //我订阅就为了求敌人在我地图的什么位置的
  target_sub_ = this->create_subscription<base_interfaces_demo::msg::Target>(
    "/track_target", 10,
    std::bind(&CalculateAttackPose::targetCallback, this, std::placeholders::_1));//接收track_target话题

  declare_parameter_if_not_declared(
    this, "chase.attack_radius", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    this, "chase.num_sectors", rclcpp::ParameterValue(36));
  declare_parameter_if_not_declared(
    this, "chase.cost_threshold", rclcpp::ParameterValue(50));
  declare_parameter_if_not_declared(
    this, "chase.costmap_topic", rclcpp::ParameterValue("global_costmap/costmap"));
  declare_parameter_if_not_declared(
    this, "chase.robot_base_frame", rclcpp::ParameterValue("chassis"));
  declare_parameter_if_not_declared(
    this, "chase.transform_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    this, "chase.max_visualization_distance", rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(
    this, "chase.marker_scale_base", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    this, "chase.visualize", rclcpp::ParameterValue(false));

  this->get_parameter("chase.attack_radius", params_.attack_radius);
  this->get_parameter("chase.num_sectors", params_.num_sectors);
  this->get_parameter("chase.cost_threshold", params_.cost_threshold);
  this->get_parameter("chase.costmap_topic", params_.costmap_topic);
  this->get_parameter("chase.robot_base_frame", params_.robot_base_frame);
  this->get_parameter("chase.transform_tolerance", params_.transform_tolerance);
  this->get_parameter("chase.max_visualization_distance", params_.max_visualization_distance);
  this->get_parameter("chase.marker_scale_base", params_.marker_scale_base);
  this->get_parameter("chase.visualize", params_.visualize);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.costmap_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CalculateAttackPose::szxnb, this, std::placeholders::_1));

  chase_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
    "/chase_point", 10);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/chase_markers", 10);

  RCLCPP_INFO(
    this->get_logger(), "Subscribing costmap topic: %s",
    params_.costmap_topic.c_str());

  timer_ = create_wall_timer(std::chrono::milliseconds(200),
    std::bind(&CalculateAttackPose::chase, this));
    

}


void CalculateAttackPose::targetCallback(base_interfaces_demo::msg::Target::SharedPtr msg)
{
  //tracker_target=msg;
  //初始化tracker
  tracker_head = msg->header.header;//敌人的时间戳和坐标系
  tracker_dis=msg->distance;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "tracker_dis: %f", tracker_dis);
  tracker_yaw=msg->yaw;
  tracking = (  tracker_dis > 0.0f &&tracker_dis<10.0f);
  //arget_received_ = true;

}


void CalculateAttackPose::szxnb(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  costmap_=msg;
}


bool CalculateAttackPose::chase()
{
  std::vector<Point> candidates;
  std::vector<Point> feasible_points;
  PoseStamped robot_pose;

  if (!tracking) {//如果没有追踪的人
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Target not tracking");

    chase_point_.x=0.0;
    chase_point_.y=0.0;
    chase_point_pub_->publish(chase_point_);//就发送（0，0）
    return false;
  }
    
  if (!costmap_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No costmap received yet");

    chase_point_.x=0.0;
    chase_point_.y=0.0;
    chase_point_pub_->publish(chase_point_);//就发送（0，0）
    return false;
  }
  


  //敌人在车体坐标系里的相对坐标frame_id是chassis
  float enemy_x_body = tracker_dis * cos(tracker_yaw);
  float enemy_y_body = tracker_dis * sin(tracker_yaw);

  //敌人相对机器人，但已经旋转到地图坐标系后的位移向量
  //float ex_map=cos(gimble_to_map_yaw) * enemy_x_body -sin(gimble_to_map_yaw) * enemy_y_body;
  //float ey_map=sin(gimble_to_map_yaw) * enemy_x_body +cos(gimble_to_map_yaw) * enemy_y_body;
  
    // Transform enemy position
    PointStamped enemy_point;
    enemy_point.header = tracker_head;//一个time 一个frame_id
    enemy_point.point.x = enemy_x_body;
    enemy_point.point.y = enemy_y_body;
    enemy_point.point.z = 0;


    if (!transformPoseInTargetFrame(//把敌人坐标放map里了，enemy_on_costmap_在此处被赋值
          enemy_point, enemy_on_costmap_, *tf_buffer_, costmap_->header.frame_id,
          params_.transform_tolerance)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to transform enemy position");//变换失败就发（0，0）
      chase_point_.x=0.0;
      chase_point_.y=0.0;
      chase_point_pub_->publish(chase_point_);//就发送（0，0）
      return false;
    }

    
    //————————————————————————————————————————
    // Get robot position (提前到候选生成之前，用于判断是否已在攻击范围内)
    if (!nav2_util::getCurrentPose(
          robot_pose, *tf_buffer_, costmap_->header.frame_id, params_.robot_base_frame,
          params_.transform_tolerance)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to get robot pose");
      chase_point_.x=0.0;
      chase_point_.y=0.0;
      chase_point_pub_->publish(chase_point_);
      return false;
    }

    // 计算机器人与敌人距离
    const double dist_to_enemy = std::hypot(
      robot_pose.pose.position.x - enemy_on_costmap_.point.x,
      robot_pose.pose.position.y - enemy_on_costmap_.point.y);

    // Generate candidate points
    candidates = generateCandidatePoints(enemy_on_costmap_.point);

    // 如果机器人在攻击范围内，把当前位置也加入候选
    if (dist_to_enemy < params_.attack_radius) {
      Point self_point;
      self_point.x = robot_pose.pose.position.x;
      self_point.y = robot_pose.pose.position.y;
      self_point.z = 0.0;
      candidates.push_back(self_point);
    }

    // Filter feasible points
    feasible_points = filterFeasiblePoints(candidates, *costmap_);

    if (feasible_points.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No feasible attack points found");

      chase_point_.x=0.0;
      chase_point_.y=0.0;
      chase_point_pub_->publish(chase_point_);//就发送（0，0）
      return false;
    }
    
    // Select best point
    const auto best_point = selectBestPoint(feasible_points, robot_pose.pose.position);
    chase_point_.x=best_point.x;
    chase_point_.y=best_point.y;

    //发布追击点
    chase_point_pub_->publish(chase_point_);



    visualization_msgs::msg::MarkerArray msg;
    // Create visualization
    if (params_.visualize) {
      createVisualizationMarkers(
        msg, enemy_on_costmap_.point, candidates, feasible_points, robot_pose.pose.position,
        *costmap_);
      for (auto & m : msg.markers){
        m.header.stamp = this->now();
      }
      marker_pub_->publish(msg);
      
    }

  return true;

}

bool CalculateAttackPose::transformPoseInTargetFrame(//将输入位姿转换为目标框架
  const PointStamped & input_pose, PointStamped & transformed_pose, tf2_ros::Buffer & tf_buffer,
  const std::string target_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    transformed_pose =
      tf_buffer.transform(input_pose, target_frame, tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(logger, "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(logger, "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger, "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(logger, "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s", input_pose.header.frame_id.c_str(),
      target_frame.c_str());
  }

  return false;
}

std::vector<Point> CalculateAttackPose::generateCandidatePoints(const Point & enemy_position)//生成候选点
{
  std::vector<Point> candidates;
  candidates.reserve(params_.num_sectors);//预分配空间，避免动态分配

  for (int i = 0; i < params_.num_sectors; ++i) {
    const double angle = i * 2 * M_PI / params_.num_sectors;
    Point p;
    p.x = enemy_position.x + params_.attack_radius * cos(angle);
    p.y = enemy_position.y + params_.attack_radius * sin(angle);
    p.z = 0.0;
    candidates.push_back(p);
  }
  return candidates;
}

std::vector<Point> CalculateAttackPose::filterFeasiblePoints(//过滤可行点
  const std::vector<Point> & candidates, const nav_msgs::msg::OccupancyGrid & costmap)
{
  std::vector<Point> feasible_points;
  const auto & info = costmap.info;

  for (const auto & p : candidates) {
    const int cell_x = static_cast<int>((p.x - info.origin.position.x) / info.resolution);
    const int cell_y = static_cast<int>((p.y - info.origin.position.y) / info.resolution);

    if (
      cell_x < 0 || cell_x >= static_cast<int>(info.width) || cell_y < 0 ||
      cell_y >= static_cast<int>(info.height)) {//判断是否超出地图范围
      continue;
    }

    const int index = cell_y * info.width + cell_x;//计算在costmap中的索引
    const int8_t cost = costmap.data[index];//获取该点的cost值
    if (cost >= 0 && cost <= params_.cost_threshold) {//判断是否为可行点
      feasible_points.push_back(p);//将可行点加入向量
    }
  }
  return feasible_points;
}

Point CalculateAttackPose::selectBestPoint(//选择最佳点
  const std::vector<Point> & feasible_points, const Point & robot_position)
{
  auto compare = [&](const auto & a, const auto & b) {
    const double dx1 = a.x - robot_position.x;
    const double dy1 = a.y - robot_position.y;
    const double dx2 = b.x - robot_position.x;
    const double dy2 = b.y - robot_position.y;
    return (dx1 * dx1 + dy1 * dy1) < (dx2 * dx2 + dy2 * dy2);
  };
  return *std::min_element(feasible_points.begin(), feasible_points.end(), compare);
}



void CalculateAttackPose::createVisualizationMarkers(//创建可视化标记
  visualization_msgs::msg::MarkerArray & msg, const Point & enemy_position,
  const std::vector<Point> & candidates, const std::vector<Point> & feasible_points,
  const Point & robot_position, const nav_msgs::msg::OccupancyGrid & costmap)
{
  msg.markers.clear();

  // Enemy marker
  visualization_msgs::msg::Marker enemy_marker;
  enemy_marker.header.frame_id = costmap.header.frame_id;
  enemy_marker.ns = "enemy";
  enemy_marker.id = 0;
  enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
  enemy_marker.pose.position = enemy_position;
  enemy_marker.scale.x = enemy_marker.scale.y = enemy_marker.scale.z = 0.3;
  enemy_marker.color.b = 1.0;
  enemy_marker.color.a = 1.0;
  msg.markers.push_back(enemy_marker);

  // Best attack marker
  const auto best_point = selectBestPoint(feasible_points, robot_position);
  visualization_msgs::msg::Marker best_marker;
  best_marker.header.frame_id = costmap.header.frame_id;
  best_marker.ns = "best";
  best_marker.id = 0;
  best_marker.type = visualization_msgs::msg::Marker::SPHERE;
  best_marker.pose.position = best_point;
  best_marker.scale.x = best_marker.scale.y = best_marker.scale.z = 0.4;
  best_marker.color.g = 1.0;
  best_marker.color.a = 1.0;
  msg.markers.push_back(best_marker);

  // Candidate markers
  const auto & info = costmap.info;
  for (size_t i = 0; i < candidates.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap.header.frame_id;
    marker.ns = "candidates";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position = candidates[i];

    // Calculate cost
    const int cell_x = (candidates[i].x - info.origin.position.x) / info.resolution;
    const int cell_y = (candidates[i].y - info.origin.position.y) / info.resolution;
    int8_t cost = -1;
    if (
      cell_x >= 0 && cell_x < static_cast<int>(info.width) && cell_y >= 0 &&
      cell_y < static_cast<int>(info.height)) {
      cost = costmap.data[cell_y * info.width + cell_x];
    }

    // Visual properties
    marker.scale.x = marker.scale.y = marker.scale.z =
      params_.marker_scale_base + (cost / 100.0) * 0.3;

    const double distance =
      std::hypot(candidates[i].x - robot_position.x, candidates[i].y - robot_position.y);
    const float alpha =
      0.5 + 0.5 * (1.0 - std::min(distance / params_.max_visualization_distance, 1.0));

    marker.color.r = 1.0;
    marker.color.a = alpha;

    // Check feasibility
    const bool is_feasible = std::any_of(
      feasible_points.begin(), feasible_points.end(),
      [&](const auto & p) { return p.x == candidates[i].x && p.y == candidates[i].y; });

    if (!is_feasible) {
      marker.color.a *= 0.3;
    }

    msg.markers.push_back(marker);
  }

  // Range circle
  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = costmap.header.frame_id;
  circle.ns = "range";
  circle.id = 0;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.pose.position.z = 0.05;
  circle.scale.x = 0.05;
  circle.color.b = 1.0;
  circle.color.a = 0.5;

  constexpr int circle_points = 36;
  for (int i = 0; i <= circle_points; ++i) {
    const double angle = i * 2 * M_PI / circle_points;
    Point p;
    p.x = enemy_position.x + params_.attack_radius * cos(angle);
    p.y = enemy_position.y + params_.attack_radius * sin(angle);
    circle.points.push_back(p);
  }
  msg.markers.push_back(circle);
}

}  // namespace qiqi2026_chase

int main(int argc,char* argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<qiqi2026_chase::CalculateAttackPose>());
  rclcpp::shutdown();
  return 0;
}
