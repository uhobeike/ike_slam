// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__IKE_SLAM_HPP_
#define IKE_SLAM__IKE_SLAM_HPP_

#include "ike_slam/pf/mcl.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace ike_slam {
class IkeSlam : public rclcpp::Node {
public:
  explicit IkeSlam(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~IkeSlam();

protected:
  void
  receiveInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                         msg); // 初期位置の受取
  void receiveScan(
      sensor_msgs::msg::LaserScan::SharedPtr msg); // LiDARからのデータの受取

  void getParam(); // パラメータを取得する

  void initPubSub(); // パブリッシャ・サブスクライバ初期化
  void initServiceServer(); // サービスサーバの初期化
  void initServiceClient(); // サービスクライアントの初期化
  void initTf();            // tf関連の初期化
  void initMcl();           // MClの初期化
  void mcl_to_ros2();       // MClからROS 2の橋渡し的なことをする
  void setParticles(
      nav2_msgs::msg::ParticleCloud
          &particles); // MCLのパーティクルからROS 2のパーティクルに置き換える
  geometry_msgs::msg::PoseStamped getMclPose(const Particle particle);

  visualization_msgs::msg::MarkerArray createSphereMarkerArray(
      const std::vector<std::pair<double, double>> particles_scan_match_point);
  void
  transformMapToOdom(); // 推定した姿勢からマップ座標系オドメトリー座標系間の変換を行う
  void setScan(const sensor_msgs::msg::LaserScan &scan);
  void getCurrentRobotPose(
      geometry_msgs::msg::PoseStamped
          &current_pose); // オドメトリー座標系でのロボット姿勢を取得する
  inline void publishParticles // ROS 2のパーティクルをパブリッシュする
      (nav2_msgs::msg::ParticleCloud particles) {
    particle_cloud_pub_->publish(particles);
  };
  inline void publishMclPose // 推定した姿勢をパブリッシュする
      (geometry_msgs::msg::PoseStamped mcl_pose) {
    mcl_pose_publisher_->publish(mcl_pose);
  };
  inline void publishMarginalLikelihood // 周辺尤度をパブリッシュする
      (const float marginal_likelihood) {
    std_msgs::msg::Float32 msg;
    msg.data = marginal_likelihood;
    marginal_likelihood_publisher_->publish(msg);
  };
  inline void
      publishParticlesScanMatchPoint // 各パーティクルのスキャンと尤度場のマッチポイントをパブリッシュする
      (const visualization_msgs::msg::MarkerArray particles_scan_match_point) {
    particles_scan_match_point_publisher_->publish(particles_scan_match_point);
  };

  void loopMcl(); // MClのループ

private:
  // サブスクライバの登録
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
      ConstSharedPtr initial_pose_sub_;

  // パブリッシャの登録
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
      particle_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      likelihood_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapping_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      particles_scan_match_point_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      marginal_likelihood_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      mcl_pose_publisher_;

  // サービスサーバの登録
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      publish_likelihoodfield_map_srv_;

  rclcpp::TimerBase::SharedPtr mcl_loop_timer_; // MClのループ用のタイマー

  rclcpp::Clock ros_clock_; // 時間を取得する用

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform latest_tf_;

  nav_msgs::msg::OccupancyGrid map_; // 受けったマップ
  sensor_msgs::msg::LaserScan scan_; // 受け取ったスキャン

  bool initialpose_receive_; // 初期位置を受け取ったかのフラグ
  bool scan_receive_;        // スキャンを受け取ったかのフラグ
  bool init_tf_;             // tfの初期化を実行したかのフラグ
  bool init_mcl_;            // MCLの初期化を実行したかのフラグ
  bool init_likelihood_map_; // 尤度場を作成したかのフラグ

  std::unique_ptr<mcl::Mcl> mcl_; // ROS依存が無いMClオブジェクト

  geometry_msgs::msg::PoseStamped current_pose_,
      past_pose_; // 現在の姿勢、現在より一個前の姿勢
  double delta_x_, delta_y_,
      delta_yaw_; // 現在の姿勢と現在の姿勢を比較したときの各差分

  Particle maximum_likelihood_particle_;

  // ike_localization用のパラメータ
  int loop_mcl_ms_;            // MClの実行周期
  double transform_tolerance_; // tfの座標変換の遅延の許容時間
  int particle_size_;          // パーティクルのサイズ
  double initial_pose_x_, initial_pose_y_,
      initial_pose_a_; // パーティクルの初期位置
  std::string map_frame_, odom_frame_, robot_frame_; // 各座標系
  double alpha1_, alpha2_, alpha3_, alpha4_; // 動作モデル用の誤差
  double likelihood_dist_;                   // 尤度場の距離
  bool
      publish_particles_scan_match_point_; // 各パーティクルのスキャンと尤度場のマッチポイントをパブリッシュするか
  float map_resolution_;                   // 作成する地図の解像度
  float icp_error_tolerance_;              // icp誤差許容値
  float icp_max_iterator_;                 // icp最大反復回数
};
} // namespace ike_slam

#endif // IKE_SLAM__IKE_SLAM_HPP_