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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <boost/circular_buffer.hpp>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabImu(const sensor_msgs::ImuConstPtr& msg);

    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;
    ros::Publisher mpFrameWithInfoPublisher;
    ORB_SLAM3::System* mpSLAM;

    double last_image_timestamp_ = 0.0;
    std::string dataset_;
    // Added by yanwei, save tracking latency
    std::vector<double> vTimesTrack;
    std::vector<pair<double, double> > vStampedTimesTrack;
    std::mutex imuMutex_;
    boost::circular_buffer<sensor_msgs::Imu> imus_{200};

    void saveStats(const std::string& path_traj)
    {
        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        int proccIm = vTimesTrack.size();
        for(int ni=0; ni<proccIm; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }

        // save to stats
        {
            std::ofstream myfile(path_traj + "_stats.txt");
            myfile << "#dummy processed_img_num mean_time median_time min_time max_time\n";
            myfile << std::setprecision(6) << -1 << " "
                << proccIm << " "
                << totaltime / proccIm << " "
                << vTimesTrack[proccIm/2] << " "
                << vTimesTrack.front() << " "
                << vTimesTrack.back() << " ";
            myfile.close();

            myfile.open(path_traj + "_Log_Latency.txt");
            myfile << "#timestamp tracking_time\n";
            myfile << std::setprecision(20);
            for (const auto& m : vStampedTimesTrack)
            {
                myfile << m.first << " " << m.second << "\n";
            }
            myfile.close();
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::start();

    // if(argc != 3)
    if (argc < 8)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD_Inertial path_to_vocabulary path_to_settings num_all_feature do_vis cam0_topic depth_topic imu_topic path_to_traj" << endl;        
        ros::shutdown();
        return 1;
    }    

    const bool do_viz = std::stoi(argv[4]);
    std::cout << "viz: " << do_viz << std::endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD,do_viz);
    SLAM.mpTracker->updateORBExtractor(stoi(argv[3]));

    // realtime trajectory logging
    std::string fNameRealTimeTrack = std::string(argv[8]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.mpTracker->SetRealTimeFileStream(fNameRealTimeTrack);

    ImageGrabber igb(&SLAM);

    if (argc > 9) {
        igb.dataset_ = argv[9];
    }

    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.subscribe(argv[7], 1000, &ImageGrabber::GrabImu, &igb); 
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[5], 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[6], 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    igb.mpCameraPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose", 100);
    igb.mpCameraPoseInIMUPublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose_in_imu", 100);

// #ifdef FRAME_WITH_INFO_PUBLISH
    igb.mpFrameWithInfoPublisher = nh.advertise<sensor_msgs::Image>("ORB_SLAM/frame_with_info", 100);
// #endif
    while(ros::ok())
        ros::spin();
 
    cout << "ros_rgbd: done with spin!" << endl;

    // save stats
    igb.saveStats(std::string(argv[8]));

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(std::string(argv[8]) + "_KeyFrameTrajectory.txt");
    SLAM.SaveTrackingLog(std::string(argv[8]) + "_Log.txt" );
    SLAM.SaveMappingLog(std::string(argv[8]) + "_Log_Mapping.txt");

    std::cout << "Finished saving!" << std::endl;
    ros::shutdown();

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}

void ImageGrabber::GrabImu(const sensor_msgs::ImuConstPtr &msg)
{
    std::unique_lock<std::mutex> lock(imuMutex_);
    imus_.push_back(*msg);
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    if (dataset_.find(std::string("gazebo")) != std::string::npos) {
        // @NOTE (yanwei) throw the first few garbage images from gazebo
        static size_t skip_imgs = 0;
        if (skip_imgs < 10)
        {
            ++skip_imgs;
            return;
        }
    }

    const double latency_trans = ros::Time::now().toSec() - msgRGB->header.stamp.toSec();

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }    // if (align_map_with_odom_)

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // retrieve imus
    double current_image_timestamp = cv_ptrRGB->header.stamp.toSec();
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
        std::unique_lock<std::mutex> lock(imuMutex_);
        int n_imus = 0;
        for (size_t i = 0; i < imus_.size(); ++i) {
            const auto& imu = imus_[i];
            double imu_timestamp = imu.header.stamp.toSec();
            // Skip imu msg that is earlier than the previous odom
            if (imu_timestamp <= last_image_timestamp_) continue;
            if (imu_timestamp > current_image_timestamp) break;
            const cv::Point3f acc(imu.linear_acceleration.x,
                                  imu.linear_acceleration.y,
                                  imu.linear_acceleration.z);
            const cv::Point3f gyr(imu.angular_velocity.x,
                                  imu.angular_velocity.y,
                                  imu.angular_velocity.z);
            vImuMeas.emplace_back(acc, gyr, imu_timestamp);
        }
        if (vImuMeas.empty()) {
            std::cout << "No imu measurements have been found between " << std::setprecision(20)
                    << last_image_timestamp_ << ", " << current_image_timestamp
                    << std::endl;
            if (imus_.size() > 0) {
                std::cout << "imu timestamp: " << std::setprecision(20) << imus_.front().header.stamp.toSec()
                        << ", " << imus_.back().header.stamp.toSec() << "\n";
            }
            return;
        }
    }

    last_image_timestamp_ = current_image_timestamp;

    auto pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(), vImuMeas);

    // collect latency
    double latency_total = ros::Time::now().toSec() - cv_ptrRGB->header.stamp.toSec();
    {
        const double track_latency = latency_total - latency_trans;
        vTimesTrack.emplace_back(track_latency);
        vStampedTimesTrack.emplace_back(cv_ptrRGB->header.stamp.toSec(), track_latency);
    }

    if (mpSLAM->mpTracker->mState != 2) // OK
    {
        std::cout << "track is NOT ready" << std::endl;
        return;
    }
    if (!(mpSLAM->mpLocalMapper->IMU_INIT_1 &&
          mpSLAM->mpLocalMapper->IMU_INIT_2))
    {
        std::cout << "IMU INIT is NOT ready" << std::endl;
        return;
    }

     // Convert the pose from Tcw to Twc.
    pose = pose.inverse();

    // by default, an additional transform is applied to make camera pose and body frame aligned
    // In inertial mode, the returned pose is in a frame with z staying vertically.
    // https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/409
    {
        const Sophus::SE3f Tbi(
            (Eigen::Matrix3f() << 0, 1, 0, -1, 0, 0, 0, 0, 1).finished(), Eigen::Vector3f::Zero()
        );
        pose = Tbi * pose;
    }

    const Eigen::Vector3f   t = pose.translation();
    const Eigen::Quaternionf q = pose.unit_quaternion();

    geometry_msgs::Pose camera_pose_in_imu;
	camera_pose_in_imu.position.x = t.x();
	camera_pose_in_imu.position.y = t.y();
	camera_pose_in_imu.position.z = t.z();
	camera_pose_in_imu.orientation.x = q.x();
	camera_pose_in_imu.orientation.y = q.y();
	camera_pose_in_imu.orientation.z = q.z();
	camera_pose_in_imu.orientation.w = q.w();

    geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
	camera_odom_in_imu.header.frame_id = "map";
	camera_odom_in_imu.header.stamp = cv_ptrRGB->header.stamp;
	camera_odom_in_imu.pose.pose = camera_pose_in_imu;

	mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);

// #ifdef FRAME_WITH_INFO_PUBLISH
    if (mpSLAM != NULL && mpSLAM->mpFrameDrawer != NULL) {
        cv::Mat fr_info_cv = mpSLAM->mpFrameDrawer->DrawFrame();
        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptrRGB->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = fr_info_cv; // Your cv::Mat
        mpFrameWithInfoPublisher.publish(out_msg.toImageMsg());
    }
// #endif

}


