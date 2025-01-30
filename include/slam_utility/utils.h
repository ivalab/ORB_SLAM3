/**
 * @file Utils.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 03-30-2024
 * @copyright Copyright (c) 2024
 */

#ifndef SLAM_UTILITY_UTILS_H_
#define SLAM_UTILITY_UTILS_H_

#include <slam_utility/types.h>

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <tuple>

namespace slam_utility {
/**
 * @brief
 *
 */
struct FramePose {
    double timestamp;

    Vec3       t;
    Quaternion q;

    /**
     * @brief Construct a new Frame Pose object
     *
     * @param _timestamp
     * @param _Tcw
     */
    FramePose(double _timestamp, const cv::Mat& _Tcw) : timestamp(_timestamp) {
        Mat44 Tcw;
        cv::cv2eigen(_Tcw, Tcw);
        const Mat33 Rwc = Tcw.topLeftCorner(3, 3).transpose();
        q               = Quaternion(Rwc);
        t               = -Rwc * Tcw.topRightCorner(3, 1);
    }

    /**
     * @brief
     *
     * @param os
     * @param f
     * @return std::ostream&
     */
    friend std::ostream& operator<<(std::ostream& os, const FramePose& f) {
        os << std::setprecision(6) << f.timestamp << std::setprecision(7) << " "
           << f.t.x() << " " << f.t.y() << " " << f.t.z() << " " << f.q.x()
           << " " << f.q.y() << " " << f.q.z() << " " << f.q.w();
        return os;
    }

    /**
     * @brief
     *
     * @return std::string
     */
    static std::string header() { return "#timestamp tx ty tz qx qy qz qw"; }
};

class Utility {
public:
    static std::pair<double, double> computePoseDiffAsDistanceAndAngle(
        const cv::Mat& cv_left, const cv::Mat& cv_right) {
        Mat44 left, right;
        cv::cv2eigen(cv_left, left);
        cv::cv2eigen(cv_right, right);
        const Mat44 diff  = left.inverse() * right;
        double      dist  = diff.topRightCorner(3, 1).norm();
        double      angle = AngleAxis(Mat33(diff.topLeftCorner(3, 3))).angle();
        return std::make_pair(dist, angle);
    }

    static std::tuple<DistributionStats, DistributionStats, DistributionStats>
    computeKeyframesDistribution(double kf_timestamp, const Pose3f& kf_pose,
                                 const std::vector<double>& timestamps,
                                 const std::vector<Pose3f>& poses) {
        std::vector<double> time_diffs;
        std::vector<double> dist_diffs;
        std::vector<double> angle_diffs;

        auto kf_pose_inv = kf_pose.inverse();
        for (size_t i = 0; i < timestamps.size(); i++) {
            time_diffs.emplace_back(std::abs(kf_timestamp - timestamps.at(i)));
            const auto diff = kf_pose_inv * poses.at(i);
            double     dist = diff.translation().norm();
            double     angle =
                AngleAxisf(Mat33f(diff.matrix().topLeftCorner(3, 3))).angle();
            dist_diffs.emplace_back(dist);
            angle_diffs.emplace_back(angle);
        }

        auto time_dist  = estimateDistributionScalar(time_diffs);
        auto dist_dist  = estimateDistributionScalar(dist_diffs);
        auto angle_dist = estimateDistributionScalar(angle_diffs);

        return std::make_tuple(time_dist, dist_dist, angle_dist);
    }

private:
    static DistributionStats estimateDistributionScalar(
        const std::vector<double>& values) {
        double n       = values.size();
        double sum     = std::accumulate(values.begin(), values.end(), 0.0);
        double mean    = sum / n;
        double std_dev = 0.0;
        for (double v : values) {
            std_dev += std::pow(v - mean, 2);
        }
        std_dev = std::sqrt(std_dev / n);
        return DistributionStats(mean, std_dev);
    }

    // static std::vector<double> computeAngles(const std::vector<Mat33f>& rots)
    // {
    //     Quaternionf avg = Quaternionf::Identity();
    //     for (const auto& rot : rots) {
    //         Quaternionf q(rot);
    //         if (q.dot(avg) < 0)
    //             q = -q;  // Ensure we average in the same hemisphere
    //         avg += q;
    //     }
    //     avg.normalize();
    //     Mat33f avg_mat = avg.toRotationMatrix().inverse();

    //     std::vector<double> angles;
    //     for (const auto& rot : rots) {
    //         Mat33f diff  = avg_mat * rot;
    //         double angle = AngleAxisf(Mat33f(diff.topLeftCorner(3,
    //         3))).angle(); angles.emplace_back(angle);
    //     }
    //     return angles;
    // }
};

}  // namespace slam_utility

#endif  // SLAM_UTILITY_UTILS_H_