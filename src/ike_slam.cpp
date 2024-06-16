// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_slam/ike_slam.hpp"

#include "ike_slam/pf/particle.hpp"

#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/string_utils.hpp>

#include <nav_msgs/srv/get_map.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ike_slam {

IkeSlam::IkeSlam(const rclcpp::NodeOptions &options)
    : Node("ike_slam", options), ros_clock_(RCL_SYSTEM_TIME),
      scan_receive_(false) {
  RCLCPP_INFO(this->get_logger(), "Run IkeSlam");

  getParam();

  initPubSub();
  initServiceServer();

  loopMcl();
}
IkeSlam::~IkeSlam() { RCLCPP_INFO(this->get_logger(), "Done IkeSlam."); }

void IkeSlam::getParam() {
  RCLCPP_INFO(get_logger(), "Run getParam.");

  this->declare_parameter("loop_mcl_hz", 10.0);
  loop_mcl_ms_ =
      1 / this->get_parameter("loop_mcl_hz").get_value<double>() * 1000;

  this->declare_parameter("transform_tolerance", 0.2);
  transform_tolerance_ =
      this->get_parameter("transform_tolerance").get_value<double>();

  this->declare_parameter("particle_size", 10);
  particle_size_ = this->get_parameter("particle_size").get_value<int>();

  this->declare_parameter("initial_pose_x", 0.0);
  this->declare_parameter("initial_pose_y", 0.0);
  this->declare_parameter("initial_pose_a", 0.0);
  initial_pose_x_ = this->get_parameter("initial_pose_x").get_value<double>();
  initial_pose_y_ = this->get_parameter("initial_pose_y").get_value<double>();
  initial_pose_a_ = this->get_parameter("initial_pose_a").get_value<double>();

  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("robot_frame", "base_footprint");
  map_frame_ = this->get_parameter("map_frame").get_value<std::string>();
  odom_frame_ = this->get_parameter("odom_frame").get_value<std::string>();
  robot_frame_ = this->get_parameter("robot_frame").get_value<std::string>();

  this->declare_parameter("alpha_trans_trans", 1.0);
  this->declare_parameter("alpha_trans_rotate", 0.2);
  this->declare_parameter("alpha_rotate_trans", 0.2);
  this->declare_parameter("alpha_rotate_rotate", 0.02);
  alpha1_ = this->get_parameter("alpha_trans_trans").get_value<double>();
  alpha2_ = this->get_parameter("alpha_trans_rotate").get_value<double>();
  alpha3_ = this->get_parameter("alpha_rotate_trans").get_value<double>();
  alpha4_ = this->get_parameter("alpha_rotate_rotate").get_value<double>();

  this->declare_parameter("likelihood_dist", 10.0);
  likelihood_dist_ = this->get_parameter("likelihood_dist").get_value<double>();

  this->declare_parameter("map_resolution", 0.05);
  map_resolution_ = this->get_parameter("map_resolution").get_value<float>();

  this->declare_parameter("icp_error_tolerance", 1.0e-2);
  icp_error_tolerance_ =
      this->get_parameter("icp_error_tolerance").get_value<double>();

  this->declare_parameter("icp_max_iterator", 100);
  icp_max_iterator_ = this->get_parameter("icp_max_iterator").get_value<int>();

  this->declare_parameter("publish_particles_scan_match_point", false);
  publish_particles_scan_match_point_ =
      this->get_parameter("publish_particles_scan_match_point")
          .get_value<bool>();

  RCLCPP_INFO(get_logger(), "Done getParam.");
}

void IkeSlam::initPubSub() {
  RCLCPP_INFO(get_logger(), "Run initPubSub.");

  particle_cloud_pub_ =
      create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", 2);
  likelihood_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "likelihood_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  mapping_map_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("mapping_map", 1);
  particles_scan_match_point_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("mcl_match", 2);
  marginal_likelihood_publisher_ =
      create_publisher<std_msgs::msg::Float32>("marginal_likelihood", 2);
  mcl_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("mcl_pose", 2);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1, std::bind(&IkeSlam::receiveScan, this, std::placeholders::_1));
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", 1,
          std::bind(&IkeSlam::receiveInitialPose, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Done initPubSub.");
}

void IkeSlam::initServiceServer() {
  auto publish_likelihoodfield_map =
      [this](
          const std::shared_ptr<rmw_request_id_t> request_header,
          [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request>
              request,
          std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    if (init_likelihood_map_) {
      likelihood_map_pub_->publish(map_);
      response->success = true;
      response->message =
          "Called /publish_likelihoodfield_map. Publish map done.";
    } else {
      response->success = false;
      response->message = "Called /publish_likelihoodfield_map.  "
                          "LikelihoodField Map has not been created yet.";
    }
  };
  publish_likelihoodfield_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "publish_likelihoodfield_map", publish_likelihoodfield_map);
}

void IkeSlam::receiveScan(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  scan_ = *msg;
  scan_receive_ = true;
}

void IkeSlam::receiveInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Run receiveInitialPose");

  mcl_->initParticles(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      tf2::getYaw(msg->pose.pose.orientation), particle_size_,
                      likelihood_dist_, map_resolution_);

  RCLCPP_INFO(get_logger(), "Done receiveInitialPose.");
};

void IkeSlam::initTf() {
  RCLCPP_INFO(get_logger(), "Run initTf.");

  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface(),
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                            false));
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  latest_tf_ = tf2::Transform::getIdentity();
  init_tf_ = true;

  RCLCPP_INFO(get_logger(), "Done initTf.");
}

// https://github.com/ros-planning/navigation2/blob/ef4de1527997c3bd813afe0c6296ff65e05700e0/nav2_amcl/src/amcl_node.cpp#L975-L1016
void IkeSlam::transformMapToOdom() {
  RCLCPP_INFO(get_logger(), "Run transformMapToOdom.");

  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, maximum_likelihood_particle_.pose.euler.yaw);
    tf2::Transform tmp_tf(
        q, tf2::Vector3(maximum_likelihood_particle_.pose.position.x,
                        maximum_likelihood_particle_.pose.position.y, 0.0));

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = robot_frame_;
    tmp_tf_stamped.header.stamp = scan_.header.stamp;

    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_);
  } catch (tf2::TransformException &e) {
    RCLCPP_DEBUG(get_logger(), "%s", e.what());
    return;
  }

  auto stamp = tf2_ros::fromMsg(scan_.header.stamp);
  tf2::TimePoint transform_expiration =
      stamp + tf2::durationFromSec(transform_tolerance_);

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = map_frame_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_;
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(),
                                             tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);

  RCLCPP_INFO(get_logger(), "Done transformMapToOdom.");
}

void IkeSlam::setScan(const sensor_msgs::msg::LaserScan &scan) {
  mcl_->scan_ = scan;
}

void IkeSlam::getCurrentRobotPose(
    geometry_msgs::msg::PoseStamped &current_pose) {
  while (rclcpp::ok() && not tf_buffer_->canTransform(odom_frame_, robot_frame_,
                                                      tf2::TimePoint())) {
    RCLCPP_WARN(get_logger(), "Wait Can Transform");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

  std_msgs::msg::Header header;
  header.set__frame_id(robot_frame_);
  header.set__stamp(rclcpp::Time());
  robot_pose.set__header(header);

  tf_buffer_->transform(robot_pose, current_pose, odom_frame_);
}

void IkeSlam::setParticles(nav2_msgs::msg::ParticleCloud &particles) {
  RCLCPP_INFO(get_logger(), "Run setParticles.");

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = ros_clock_.now();
  particles.set__header(header);

  particles.particles.resize(particle_size_);
  for (auto i = 0; i < particle_size_; ++i) {
    particles.particles[i].pose.position.x =
        mcl_->particles_[i].pose.position.x;
    particles.particles[i].pose.position.y =
        mcl_->particles_[i].pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, mcl_->particles_[i].pose.euler.yaw);
    particles.particles[i].pose.orientation = tf2::toMsg(q);

    particles.particles[i].weight = mcl_->particles_[i].weight;
  }

  RCLCPP_INFO(get_logger(), "Done setParticles.");
}

geometry_msgs::msg::PoseStamped IkeSlam::getMclPose(const Particle particle) {
  RCLCPP_INFO(get_logger(), "Run getMclPose.");

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = ros_clock_.now();

  geometry_msgs::msg::Pose pose;
  pose.position.x = particle.pose.position.x;
  pose.position.y = particle.pose.position.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, particle.pose.euler.yaw);
  pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::PoseStamped mcl_pose;
  mcl_pose.set__header(header);
  mcl_pose.set__pose(pose);

  RCLCPP_INFO(get_logger(), "Done getMclPose.");

  return mcl_pose;
}

visualization_msgs::msg::MarkerArray IkeSlam::createSphereMarkerArray(
    const std::vector<std::pair<double, double>> particles_scan_match_point) {
  int id = 0;
  std::string name = "";
  std_msgs::msg::Header header;
  header.frame_id = map_frame_;
  header.stamp = ros_clock_.now();

  auto marker_array = visualization_msgs::msg::MarkerArray();
  for (auto hit_xy : particles_scan_match_point) {
    auto marker = visualization_msgs::msg::Marker();

    marker.set__header(header);
    marker.ns = name;
    marker.id = id;
    id++;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.;
    marker.color.r = 1.;
    marker.color.g = 0.;
    marker.color.b = 0.;

    marker.pose.position.x = hit_xy.first;
    marker.pose.position.y = hit_xy.second;
    marker.pose.position.z = 0.2;

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

void IkeSlam::initMcl() {
  RCLCPP_INFO(get_logger(), "Run initMcl.");

  mcl_.reset();
  mcl_ = std::make_unique<mcl::Mcl>(
      initial_pose_x_, initial_pose_y_, initial_pose_a_, alpha1_, alpha2_,
      alpha3_, alpha4_, particle_size_, likelihood_dist_, map_resolution_,
      publish_particles_scan_match_point_);

  likelihood_map_pub_->publish(map_);
  mcl_->initParticles(initial_pose_x_, initial_pose_y_, initial_pose_a_,
                      particle_size_, likelihood_dist_, map_resolution_);
  maximum_likelihood_particle_.pose.position.x = initial_pose_x_;
  maximum_likelihood_particle_.pose.position.y = initial_pose_y_;
  maximum_likelihood_particle_.pose.euler.yaw = initial_pose_a_;

  init_mcl_ = true;
  init_likelihood_map_ = true;

  RCLCPP_INFO(get_logger(), "Done initMcl.");
}

void IkeSlam::mcl_to_ros2() {
  RCLCPP_INFO(get_logger(), "Run mcl_to_ros2.");

  nav2_msgs::msg::ParticleCloud particles;
  transformMapToOdom();
  setParticles(particles);
  publishParticles(particles);
  publishMclPose(getMclPose(maximum_likelihood_particle_));
  publishMarginalLikelihood(mcl_->getMarginalLikelihood());
  if (publish_particles_scan_match_point_)
    publishParticlesScanMatchPoint(
        createSphereMarkerArray(mcl_->getParticlesScanMatchPoint()));

  RCLCPP_INFO(get_logger(), "Done mcl_to_ros2.");
}

void IkeSlam::loopMcl() {
  mcl_loop_timer_ =
      create_wall_timer(std::chrono::milliseconds{loop_mcl_ms_}, [this]() {
        if (rclcpp::ok() && scan_receive_ && init_tf_ && init_mcl_) {
          RCLCPP_INFO(get_logger(), "Run IkeSlam::loopMcl");
          setScan(scan_);
          getCurrentRobotPose(current_pose_);

          mcl_->mapping_->gridMapping(mcl_->particles_, mcl_->scan_);
          auto map = mcl_->particles_[0].map->smap_.toOccupancyGrid();
          map.header.frame_id = map_frame_;
          map.info.resolution = map_resolution_;
          map.info.origin.position.x =
              map.info.origin.position.x * map_resolution_;
          map.info.origin.position.y =
              map.info.origin.position.y * map_resolution_;
          mapping_map_pub_->publish(map);

          mcl_->motion_model_->getDelta(
              delta_x_, delta_y_, delta_yaw_, current_pose_.pose.position.x,
              past_pose_.pose.position.x, current_pose_.pose.position.y,
              past_pose_.pose.position.y,
              tf2::getYaw(current_pose_.pose.orientation),
              tf2::getYaw(past_pose_.pose.orientation));

          mcl_->motion_model_->update(
              mcl_->particles_, tf2::getYaw(current_pose_.pose.orientation),
              delta_x_, delta_y_, delta_yaw_);

          for (auto &p : mcl_->particles_) {
            auto [R, t] = mcl_->scan_matching_->icp(
                p.map->scan_hit_cells_, p.map->smap_.toPointCloud(),
                icp_max_iterator_, icp_error_tolerance_);
            Eigen::Vector2d pos(p.pose.position.x, p.pose.position.y);
            Eigen::Vector2d updated_pos = R * pos + t;

            double theta = std::atan2(R(1, 0), R(0, 0));
            p.pose.euler.yaw += theta;

            p.pose.position.x = updated_pos(0);
            p.pose.position.y = updated_pos(1);
          }

          mcl_->observation_model_->update(mcl_->particles_, mcl_->scan_);

          mcl_->resampling_->resampling(mcl_->particles_);

          mcl_->getMeanParticle(maximum_likelihood_particle_);
          past_pose_ = current_pose_;

          mcl_to_ros2();
        } else {
          if (!init_tf_)
            initTf();
          if (init_tf_) {
            getCurrentRobotPose(current_pose_);
            past_pose_ = current_pose_;
          }
          if (scan_receive_ && !init_mcl_)
            initMcl();
          if (not scan_receive_)
            RCLCPP_WARN(
                get_logger(),
                "Not yet received scan. Therefore, MCL cannot be initiated.");
        }
      });
}
} // namespace ike_slam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ike_slam::IkeSlam)
