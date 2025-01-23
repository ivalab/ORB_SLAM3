/**
 * @file Stats.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 03-30-2024
 * @copyright Copyright (c) 2024
 */

#ifndef SLAM_UTILITY_STATS_H_
#define SLAM_UTILITY_STATS_H_

#include <iomanip>
#include <iostream>

namespace slam_utility {

struct DistributionStats {
    double mean = 0.0;
    double std  = -1.0;

    DistributionStats(double _mean = 0.0, double _std = -1.0)
        : mean(_mean), std(_std) {}

    bool isValid() { return std >= 0.0; }

    friend std::ostream& operator<<(std::ostream&            os,
                                    const DistributionStats& d) {
        os << std::setprecision(6);
        os << d.mean << " " << d.std;
        return os;
    }
};

struct OptimizerStats {
    double timestamp    = 0.0;
    double pre_opt      = 0.0;
    double opt          = 0.0;
    double acquire_lock = 0.0;
    double post_opt     = 0.0;

    void setZero() {
        timestamp    = 0.0;
        pre_opt      = 0.0;
        opt          = 0.0;
        acquire_lock = 0.0;
        post_opt     = 0.0;
    }

    friend std::ostream& operator<<(std::ostream& os, const OptimizerStats& o) {
        os << std::setprecision(10);
        os << o.timestamp << " " << o.pre_opt << " " << o.opt << " "
           << o.acquire_lock << " " << o.post_opt;
        return os;
    }

    static std::string header() {
        return "# timestamp pre_opt opt acquire_lock post_opt";
    }
};

/**
 * @brief
 *
 */
struct LocalMappingStats {
    double timestamp        = 0.0;
    double search_neighbors = 0.0;
    double local_ba         = 0.0;
    int    num_covis_kfs    = 0.0;
    int    num_fixed_kfs    = 0;
    int    num_opt_kfs      = 0;
    int    num_points       = 0;
    int    num_edges        = 0;

    /**
     * @brief Set the Zero object
     *
     */
    void setZero() {
        timestamp        = 0.0;
        search_neighbors = 0.0;
        local_ba         = 0.0;
        num_covis_kfs    = 0;
        num_fixed_kfs    = 0;
        num_opt_kfs      = 0;
        num_points       = 0;
        num_edges        = 0;
    }

    friend std::ostream& operator<<(std::ostream&            os,
                                    const LocalMappingStats& l) {
        os << std::setprecision(10);
        os << l.timestamp << " " << l.search_neighbors << " " << l.local_ba
           << " " << l.num_covis_kfs << " " << l.num_fixed_kfs << " "
           << l.num_opt_kfs << " " << l.num_points << " " << l.num_edges << " ";
        return os;
    }

    /**
     * @brief
     *
     * @return std::string
     */
    static std::string header() {
        return "# timestamp search_neighbors local_ba num_covis_kfs "
               "num_fixed_kfs "
               "num_opt_kfs num_points "
               "num_edges";
    }
};

/**
 * @brief
 *
 */
struct TrackingStats {
    double timestamp          = 0.0;
    double feature_extraction = 0.0;
    double stereo_matching    = 0.0;
    double create_frame       = 0.0;
    double track_motion       = 0.0;
    double track_keyframe     = 0.0;
    double track_map          = 0.0;
    double update_motion      = 0.0;
    double post_processing    = 0.0;
    double object_matching    = 0.0;
    double object_graph       = 0.0;
    double all                = 0.0;

    /**
     * @brief Set the Zero object
     *
     */
    void setZero() {
        timestamp          = 0.0;
        feature_extraction = 0.0;
        stereo_matching    = 0.0;
        create_frame       = 0.0;
        track_motion       = 0.0;
        track_keyframe     = 0.0;
        track_map          = 0.0;
        update_motion      = 0.0;
        post_processing    = 0.0;
        object_matching    = 0.0;
        object_graph       = 0.0;
        all                = 0.0;
    }

    friend std::ostream& operator<<(std::ostream& os, const TrackingStats& l) {
        os << std::setprecision(10);
        os << l.timestamp << " " << l.feature_extraction << " "
           << l.stereo_matching << " " << l.create_frame << " "
           << l.track_motion << " " << l.track_keyframe << " " << l.track_map
           << " " << l.update_motion << " " << l.post_processing << " "
           << l.object_matching << " " << l.object_graph << " " << l.all;
        return os;
    }

    static std::string header() {
        return "# timestamp feature_extraction stereo_matching "
               "create_frame track_motion track_keyframe track_map "
               "update_motion post_processing object_matching object_graph all";
    }
};

struct CovisibilityStats {
    double timestamp{-1.0};
    int    num_covis_kfs{0};

    DistributionStats time_dist{};
    DistributionStats dist_dist{};
    DistributionStats angle_dist{};
    // std::vector<std::pair<double, double> > dists_angles;
    friend std::ostream& operator<<(std::ostream&            os,
                                    const CovisibilityStats& cs) {
        os << std::setprecision(10);
        os << cs.timestamp << " " << cs.num_covis_kfs << " " << cs.time_dist
           << " " << cs.dist_dist << " " << cs.angle_dist;
        return os;
    }

    static std::string header() {
        return "# timestamp num_covis_kfs time-dist(mean, std) "
               "distance-distribution(mean, std) angle-distribution(mean, std)";
    }
};

struct Map2FrameStats {
    double            timestamp        = 0.0;
    double            ref_kf_timestamp = 0.0;
    int               max_observed_pts = 0.0;
    int               num_kfs          = 0;
    int               num_pts          = 0;
    DistributionStats time_dist{};
    DistributionStats dist_dist{};
    DistributionStats angle_dist{};

    void setZero() { *this = Map2FrameStats(); }

    friend std::ostream& operator<<(std::ostream& os, const Map2FrameStats& m) {
        os << std::setprecision(10);
        os << m.timestamp << " " << m.ref_kf_timestamp << " "
           << m.max_observed_pts << " " << m.num_kfs << " " << m.num_pts << " "
           << m.time_dist << " " << m.dist_dist << " " << m.angle_dist;
        return os;
    }

    static std::string header() {
        return "# timestamp ref-kf-timestamp max-observed-pts num_kfs num_pts "
               "time-dist(mean, std) distance-distribution(mean, std) "
               "angle-distribution(mean, std)";
    }
};

}  // namespace slam_utility

#endif  // ORB_SLAM2_STATS_H_