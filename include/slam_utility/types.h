/**
 * @file types.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 03-30-2024
 * @copyright Copyright (c) 2024
 */

#ifndef SLAM_UTILITY_TYPES_H_
#define SLAM_UTILITY_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/optional.hpp>
#include <sophus/se3.hpp>

namespace slam_utility {

using Mat44       = Eigen::Matrix4d;
using Mat33       = Eigen::Matrix3d;
using Vec3        = Eigen::Vector3d;
using Quaternion  = Eigen::Quaterniond;
using AngleAxis   = Eigen::AngleAxisd;
using Mat44f      = Eigen::Matrix4f;
using Mat33f      = Eigen::Matrix3f;
using Vec3f       = Eigen::Vector3f;
using Quaternionf = Eigen::Quaternionf;
using AngleAxisf  = Eigen::AngleAxisf;
using Pose3       = Sophus::SE3d;
using Pose3f      = Sophus::SE3f;

using OptionalMat44  = boost::optional<Mat44>;
using OptionalPose3f = boost::optional<Pose3f>;

}  // namespace slam_utility

namespace su = slam_utility;

#endif  // SLAM_UTILITY_TYPES_H_