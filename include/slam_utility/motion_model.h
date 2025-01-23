/**
 * @file motion_model.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 01-08-2025
 * @copyright Copyright (c) 2025
 */

#ifndef SLAM_UTILITY_MOTION_MODEL_H_
#define SLAM_UTILITY_MOTION_MODEL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <vector>

#include "slam_utility/types.h"

namespace slam_utility {

struct StampedPose {
    double timestamp{-1.0};
    double tx{0.0}, ty{0.0}, tz{0.0};
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};

    StampedPose(double _t = -1.0, double _tx = 0.0, double _ty = 0.0,
                double _tz = 0.0, double _qx = 0.0, double _qy = 0.0,
                double _qz = 0.0, double _qw = 1.0)
        : timestamp(_t),
          tx(_tx),
          ty(_ty),
          tz(_tz),
          qx(_qx),
          qy(_qy),
          qz(_qz),
          qw(_qw) {}

    bool operator<(const StampedPose& Twc) const {
        return this->timestamp < Twc.timestamp;
    }
    Mat44 Tmb() const {
        Mat44 mat               = Mat44::Identity();
        mat.topLeftCorner(3, 3) = Quaternion(qw, qx, qy, qz).toRotationMatrix();
        mat.topRightCorner(3, 1) = Vec3(tx, ty, tz);
        return mat;
    }
    Mat44                Tbm() const { return Tmb().inverse(); }
    friend std::ostream& operator<<(std::ostream& os, const StampedPose& p) {
        os << std::setprecision(9) << p.timestamp << " " << std::setprecision(6)
           << p.tx << " " << p.ty << " " << p.tz << " " << p.qx << " " << p.qy
           << " " << p.qz << " " << p.qw;
        return os;
    }
};
using OptionalStampedPose = boost::optional<StampedPose>;

class MotionModelBase {
public:
    using Ptr        = std::shared_ptr<MotionModelBase>;
    using ConstPtr   = std::shared_ptr<const MotionModelBase>;
    using BufferType = boost::circular_buffer<StampedPose>;
    // using BufferType = std::vector<StampedPose>;

    MotionModelBase() : buffer_(5000) {}
    virtual ~MotionModelBase() {}

    /**
     * @brief
     *
     * @param query_t
     * @param tol
     * @return OptionalStampedPose
     */
    OptionalStampedPose predict(double query_t, double tol = 1e-1) {
        std::unique_lock<std::mutex> lock(mutex_);
        return interpolatePose(query_t, tol);
    }

    /**
     * @brief
     *
     * @param odom
     */
    void append(const StampedPose& pose) {
        std::unique_lock<std::mutex> lock(mutex_);
        buffer_.push_back(pose);
    }

    const BufferType& GetBuffer() const { return buffer_; }

protected:
    OptionalStampedPose interpolatePose(double query_time, double tol = 1e-1) {
        if (buffer_.empty()) {
            // std::cerr << "The buffer is empty." << "\n";
            return boost::none;
        }

        // Find the closest poses for interpolation
        auto upper =
            std::upper_bound(buffer_.begin(), buffer_.end(), query_time,
                             [](double t, const StampedPose& pose) {
                                 return t < pose.timestamp;
                             });
        if (upper == buffer_.begin()) {
            // Query time is earlier than the first timestamp
            if (std::abs(query_time - buffer_.front().timestamp) <= tol) {
                return buffer_.front();
            } else {
                std::cerr
                    << "Query time is out of range and exceeds the tolerance."
                    << "\n";
                return boost::none;
            }
        }

        if (upper == buffer_.end()) {
            // Query time is later than the last timestamp
            if (std::abs(query_time - buffer_.back().timestamp) <= tol) {
                return buffer_.back();
            } else {
                std::cerr
                    << "Query time is out of range and exceeds the tolerance."
                    << "\n";
                return boost::none;
            }
        }

        // Get lower.
        auto lower = std::prev(upper);
        // Interpolate between lower and upper
        const StampedPose& pose1 = *lower;
        const StampedPose& pose2 = *upper;
        StampedPose        interpolated;
        interpolated.timestamp = query_time;
        interpolated.tx = linearInterpolate(pose1.tx, pose2.tx, pose1.timestamp,
                                            pose2.timestamp, query_time);
        interpolated.ty = linearInterpolate(pose1.ty, pose2.ty, pose1.timestamp,
                                            pose2.timestamp, query_time);
        interpolated.tz = linearInterpolate(pose1.tz, pose2.tz, pose1.timestamp,
                                            pose2.timestamp, query_time);
        Quaternion q1(pose1.qw, pose1.qx, pose1.qy, pose1.qz);
        Quaternion q2(pose2.qw, pose2.qx, pose2.qy, pose2.qz);
        Quaternion interpolated_q = slerpInterpolate(
            q1, q2, pose1.timestamp, pose2.timestamp, query_time);
        interpolated.qw = interpolated_q.w();
        interpolated.qx = interpolated_q.x();
        interpolated.qy = interpolated_q.y();
        interpolated.qz = interpolated_q.z();
        return interpolated;
    }

    /**
     * @brief
     *
     * @param v1
     * @param v2
     * @param t1
     * @param t2
     * @param t_query
     * @return double
     */
    double linearInterpolate(double v1, double v2, double t1, double t2,
                             double t_query) {
        return v1 + (v2 - v1) * ((t_query - t1) / (t2 - t1));
    }

    /**
     * @brief
     *
     * @param q1
     * @param q2
     * @param t1
     * @param t2
     * @param t_query
     * @return Quaternion
     */
    Quaternion slerpInterpolate(const Quaternion& q1, const Quaternion& q2,
                                double t1, double t2, double t_query) {
        double alpha = (t_query - t1) / (t2 - t1);
        return q1.slerp(alpha, q2);
    }

    std::mutex mutex_;
    BufferType buffer_;
};

class MotionModel : public MotionModelBase {
public:
    using Ptr = std::shared_ptr<MotionModel>;

    /**
     * @brief Construct a new Motion Model object
     *
     */
    MotionModel() : MotionModelBase() {
        // Default set to gazebo settings.
        Toc_.setQuaternion(Quaternionf(-0.500, 0.500, -0.500, 0.500));
        Toc_.translation() = Vec3f(0.094, -0.074, 0.280);
        Tco_               = Toc_.inverse();
    }

    /**
     * @brief Construct a new Motion Model object
     *
     * @param odom_file
     * @param Toc
     */
    MotionModel(const std::string& odom_file, const Pose3f& Toc)
        : MotionModelBase() {
        loadOdom(odom_file, Toc);
    }

    virtual ~MotionModel() {}

    /**
     * @brief
     *
     * @param t1
     * @param t2
     * @param tol
     * @return Mat44
     */
    OptionalPose3f predictRelMotionORB(double t1, double t2,
                                       double tol = 1e-1) {
        auto T1 = MotionModelBase::predict(t1, tol);
        if (!T1) {
            return boost::none;
        }
        auto T2 = MotionModelBase::predict(t2, tol);
        if (!T2) {
            return boost::none;
        }
        const Pose3f To_21 = Pose3f(Mat44(T2->Tbm() * T1->Tmb()).cast<float>());
        const Pose3f Tc_21 = Tco_ * To_21 * Toc_;
        return Tc_21;
    }

    /**
     * @brief
     *
     * @param odom_file
     */
    void loadOdom(const std::string& odom_file, const Pose3f& Toc) {
        std::ifstream myfile(odom_file);
        if (!myfile.is_open()) {
            throw std::runtime_error("Invalid Odom file.");
        }

        size_t                       count = 0;
        std::string                  line;
        StampedPose                  p;
        std::unique_lock<std::mutex> lock(mutex_);
        while (std::getline(myfile, line)) {
            //            cout << line << endl;
            std::istringstream iss(line);
            double             vx, vy, vz, wx, wy, wz;
            if (!(iss >> p.timestamp >> p.tx >> p.ty >> p.tz >> p.qx >> p.qy >>
                  p.qz >> p.qw >> vx >> vy >> vz >> wx >> wy >> wz)) {
                break;
            }
            // auto it = std::lower_bound(buffer_.begin(), buffer_.end(), p);
            // buffer_.insert(it, p);
            buffer_.push_back(p);
            count++;
        }
        Toc_ = Toc;
        Tco_ = Toc.inverse();
        std::cout << "Loaded " << count << " wheel odom records" << std::endl;
    };

private:
    Pose3f Toc_;
    Pose3f Tco_;
};

}  // namespace slam_utility

#endif  //