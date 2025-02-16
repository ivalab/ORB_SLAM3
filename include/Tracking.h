/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"
#include "MacroDefinitions.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

#include <slam_utility/timer.h>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;


class OdometryLog
{
public:

  OdometryLog(double time_stamp_ = -1.0, double tx_ = 0.0, double ty_ = 0.0, double tz_ = 0.0,
	  double qw_ = 1.0, double qx_ = 0.0, double qy_ = 0.0, double qz_ = 0.0) :
      time_stamp(time_stamp_), tx(tx_), ty(ty_), tz(tz_), qw(qw_), qx(qx_), qy(qy_), qz(qz_),
      Twc(Eigen::Quaternionf(qw_, qx_, qy_, qz_), Eigen::Vector3f(tx_, ty_, tz_)) {
    Tcw = Twc.inverse();
  }

  double time_stamp;
  double tx, ty, tz;
  double qw, qx, qy, qz;

  Sophus::SE3f Tcw;
  Sophus::SE3f Twc;

};

struct OdomLogComparator {
    bool operator()(double const& t0, OdometryLog const& m1) const {
        return t0 < m1.time_stamp;
    }

    bool operator()(OdometryLog const& m0, double const& t1) const {
        return m0.time_stamp < t1;
    }
};
class MotionModel
{
public:
    MotionModel()
    {
        buffer_.reserve(10000);
        // Initialize extrisnics.
        Tc2b.setQuaternion(Eigen::Quaternion<float>(-0.500, 0.500, -0.500, 0.500));
        Tc2b.translation() = Eigen::Vector3f(0.094, -0.074, 0.280);
        Tb2c = Tc2b.inverse();
    }

    void append(const OdometryLog& odom)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        buffer_.emplace_back(odom);
    }

    void reset()
    {
        buffer_.clear();
    }

    bool predict(const double time_prev, const double time_curr, Sophus::SE3f & T_se) {
        std::unique_lock<std::mutex> lock(mutex_);
        int n = buffer_.size(), i;
        //cout << "mvOdomBuf.size() = " << n ;

        auto lower = std::lower_bound( buffer_.begin(), buffer_.end(), time_prev, OdomLogComparator() );
        auto upper = std::upper_bound( buffer_.begin(), buffer_.end(), time_curr, OdomLogComparator() );

        if (lower == buffer_.end()){
            buffer_.clear();
            return false;
        }
        const auto T_st = lower->Twc;

        if (upper == buffer_.end()){
            buffer_.clear();
            return false;
        }
        const auto T_ed = upper->Twc;

        // relative transform between i_st & i_ed
        // cv::Mat Ttmp = (Tb2c * T_ed * T_st.inv() * Tc2b);
        T_se = (Tb2c * T_ed.inverse() * T_st * Tc2b);

        //mvOdomBuf.clear();
        buffer_.erase(buffer_.begin(), upper);
        return true;
    }

private:
    std::mutex mutex_;
    std::vector<OdometryLog> buffer_;
    Sophus::SE3f Tb2c, Tc2b;
};
class Tracking
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq=std::string());

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();
    void updateORBExtractor(int feature_num);
    void SetRealTimeFileStream(string fNameRealTimeTrack);

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;


    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    //void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // System
    System* mpSystem;

    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    //Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    Sophus::SE3f mTlr;

    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

    std::ofstream f_realTimeTrack;

public:
    cv::Mat mImRight;

    struct TimeLog
    {
        double timestamp          = 0.0;
        double feature_extraction = 0.0;
        double stereo_matching    = 0.0;
        double create_frame       = 0.0;
        double track_motion       = 0.0;
        double track_keyframe     = 0.0;
        double track_map          = 0.0;
        double update_motion      = 0.0;
        double post_processing    = 0.0;

        /**
         * @brief Set the Zero object
         *
         */
        void setZero()
        {
            timestamp          = 0.0;
            feature_extraction = 0.0;
            stereo_matching    = 0.0;
            create_frame       = 0.0;
            track_motion       = 0.0;
            track_keyframe     = 0.0;
            track_map          = 0.0;
            update_motion      = 0.0;
            post_processing    = 0.0;
        }

        friend std::ostream& operator<<(std::ostream& os, const TimeLog& l)
        {
            os << std::setprecision(10);
            os << l.timestamp << " "
               << l.feature_extraction << " "
               << l.stereo_matching << " "
               << l.create_frame << " "
               << l.track_motion << " "
               << l.track_keyframe << " "
               << l.track_map << " "
               << l.update_motion << " "
               << l.post_processing;
            return os;
        }

        static std::string header()
        {
            return "# timestamp feature_extraction stereo_matching "
                   "create_frame track_motion track_keyframe track_map "
                   "update_motion post_processing";
        }
    };
    TimeLog logCurrentFrame_;
    std::vector<TimeLog> mFrameTimeLog_;
    slam_utility::TicTocTimer timer_;

    MotionModel motion_model_;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
