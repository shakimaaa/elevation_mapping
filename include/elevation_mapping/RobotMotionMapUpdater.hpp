/*
 * RobotMotionMapUpdater.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/Core>

// ROS
#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {

/*!
 * 根据机器人的位姿协方差计算地图方差更新。
 */
class RobotMotionMapUpdater {
 public:
  using Pose = kindr::HomogeneousTransformationPosition3RotationQuaternionD;
  using Covariance = Eigen::Matrix<double, 3, 3>;
  using PoseCovariance = Eigen::Matrix<double, 6, 6>;
  using ReducedCovariance = Eigen::Matrix<double, 4, 4>;
  using Jacobian = Eigen::Matrix<double, 4, 4>;

  /*!
   * 构造函数。
   */
  explicit RobotMotionMapUpdater(std::shared_ptr<rclcpp::Node> nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RobotMotionMapUpdater();

  /*!
   * 读取并验证 ROS 参数。
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * 根据位姿协方差计算高程图的模型更新
   *添加更新到地图。
   * @param[in] map 要更新的高程图。
   * @param[in] robotPose 当前的姿势。
   * @param[in] robotPoseCovariance 当前姿态协方差矩阵。
   * @param[in] time 当前更新的时间。
   * @return true if successful.
   */
  bool update(ElevationMap& map, const Pose& robotPose, const PoseCovariance& robotPoseCovariance, const rclcpp::Time& time);

 private:
  /*!
   * 根据完整姿态协方差（6x6：x、y、z、roll、pitch、yaw）计算简化协方差（4x4：x、y、z、yaw）。
   * @param[in] robotPose 机器人的姿势。
   * @param[in] robotPoseCovariance 全姿态协方差矩阵 (6x6)。
   * @param[out] reducedCovariance 简化协方差矩阵 (4x4)；
   * @return true if successful.
   */
  bool computeReducedCovariance(const Pose& robotPose, const PoseCovariance& robotPoseCovariance, ReducedCovariance& reducedCovariance);

  /*!
   * 计算新姿势和前一个姿势之间的协方差。
   * @param[in] robotPose 当前的机器人姿势。
   * @param[in] reducedCovariance 当前机器人位姿协方差矩阵（简化）。
   * @param[out] relativeRobotPoseCovariance 当前和前一个机器人姿势之间的相对协方差（简化形式）。
   * @return true if successful.
   */
  bool computeRelativeCovariance(const Pose& robotPose, const ReducedCovariance& reducedCovariance, ReducedCovariance& relativeCovariance);

  //! ROS 节点句柄。
  std::shared_ptr<rclcpp::Node> nodeHandle_;

  //! 上次更新时间。
  rclcpp::Time previousUpdateTime_;

  //! 之前的机器人姿势。
  Pose previousRobotPose_;

  //! 与上次更新相比，机器人姿势协方差（减少）。
  ReducedCovariance previousReducedCovariance_;

  //! 协方差矩阵的缩放因子（默认为 1）。
  double covarianceScale_;
};

}  // namespace elevation_mapping
