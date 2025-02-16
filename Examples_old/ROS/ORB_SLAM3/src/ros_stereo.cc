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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/SetBool.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

//
//#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    void GrabOdom(const nav_msgs::OdometryConstPtr &msg);

    bool HandlePauseRequest(std_srvs::SetBool::Request&  req,
                            std_srvs::SetBool::Response& res);

    ORB_SLAM3::System* mpSLAM;

    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;
    //    ros::Publisher mpDensePathPub;

    ros::ServiceServer service;
    bool paused = false;

#ifdef MAP_PUBLISH
    size_t mnMapRefreshCounter;
    ORB_SLAM2::MapPublisher* mpMapPub;
#endif

// #ifdef FRAME_WITH_INFO_PUBLISH
    ros::Publisher mpFrameWithInfoPublisher;
// #endif

   // Added by yanwei, save tracking latency
    std::vector<double> vTimesTrack;
    std::vector<pair<double, double> > vStampedTimesTrack;

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
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc < 8)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings num_all_feature do_rectify do_vis cam0_topic cam1_topic path_to_traj" << endl;
        ros::shutdown();
        return 1;
    }

    const bool do_viz = std::stoi(argv[5]);
    std::cout << "viz: " << do_viz << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, do_viz);
    SLAM.mpTracker->updateORBExtractor(stoi(argv[3]));

    // realtime trajectory logging
    std::string fNameRealTimeTrack = std::string(argv[8]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.mpTracker->SetRealTimeFileStream(fNameRealTimeTrack);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    // message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, argv[6], 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, argv[7], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(2), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    igb.mpCameraPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose", 100);
    igb.mpCameraPoseInIMUPublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose_in_imu", 100);
    ros::Subscriber sub = nh.subscribe("/odom_dummy", 100, &ImageGrabber::GrabOdom, &igb);

    // Service to pause the node.
    igb.service = nh.advertiseService("/pause_slam", &ImageGrabber::HandlePauseRequest, &igb);
    igb.paused = false;

// #ifdef FRAME_WITH_INFO_PUBLISH
    igb.mpFrameWithInfoPublisher = nh.advertise<sensor_msgs::Image>("ORB_SLAM/frame_with_info", 100);
// #endif
    while(ros::ok())
        ros::spin();
    // ros::spin();

    cout << "ros_stereo: done with spin!" << endl;

    // save stats
    igb.saveStats(std::string(argv[8]));

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(std::string(argv[8]) + "_KeyFrameTrajectory.txt");
    SLAM.SaveTrackingLog(std::string(argv[8]) + "_Log.txt" );
    SLAM.SaveMappingLog(std::string(argv[8]) + "_Log_Mapping.txt");
    // SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    // SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    std::cout << "Finished saving!" << std::endl;

    ros::shutdown();

    cout << "ros_stereo: done with ros Shutdown!" << endl;

    // Stop all threads
    SLAM.Shutdown();
    cout << "ros_stereo: done with SLAM Shutdown!" << endl;

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
#ifdef ENABLE_CLOSED_LOOP
    // @NOTE (yanwei) throw the first few garbage images from gazebo
    static size_t skip_imgs = 0;
    if (skip_imgs < 10)
    {
        ++skip_imgs;
        return;
    }
#endif

    if (paused) {
        ROS_WARN_STREAM_THROTTLE(10.0, "ORB3 has paused, waiting to resume ...");
        return;
    }

    const double latency_trans = ros::Time::now().toSec() - msgLeft->header.stamp.toSec();

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // tracking
    Sophus::SE3f pose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());

    // collect latency
    double latency_total = ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec();
    {
        const double track_latency = latency_total - latency_trans;
        vTimesTrack.emplace_back(track_latency);
        vStampedTimesTrack.emplace_back(cv_ptrLeft->header.stamp.toSec(), track_latency);
    }

    if (mpSLAM->mpTracker->mState != 2) // OK
    {
        return;
    }

// #ifdef ENABLE_CLOSED_LOOP
//     ROS_INFO("Pose Tracking Latency: %.03f sec", latency_total - latency_trans);
// #endif

    // prepare for publishing
    const Sophus::SE3f Twc = pose.inverse();
    const Eigen::Matrix<float, 3, 3> Rwc = Twc.rotationMatrix();
    const Eigen::Matrix<float, 3, 1> twc = Twc.translation();

    tf::Matrix3x3 M(
        Rwc(0,0), Rwc(0,1), Rwc(0,2),
		Rwc(1,0), Rwc(1,1), Rwc(1,2),
		Rwc(2,0), Rwc(2,1), Rwc(2,2));
    tf::Vector3 V(twc(0), twc(1), twc(2));

    tf::Transform tfTcw(M,V);
    geometry_msgs::Transform gmTwc;
    tf::transformTFToMsg(tfTcw, gmTwc);

    geometry_msgs::Pose camera_pose;
    camera_pose.position.x = gmTwc.translation.x;
    camera_pose.position.y = gmTwc.translation.y;
    camera_pose.position.z = gmTwc.translation.z;
    camera_pose.orientation = gmTwc.rotation;

    geometry_msgs::PoseWithCovarianceStamped camera_odom;
    camera_odom.header.frame_id = "odom";
    camera_odom.header.stamp = cv_ptrLeft->header.stamp;
    camera_odom.pose.pose = camera_pose;

    mpCameraPosePublisher.publish(camera_odom);

//
// by default, an additional transform is applied to make camera pose and body frame aligned
// which is assumed in msf
#ifdef INIT_WITH_ARUCHO
    tf::Matrix3x3 Ric(   0, -1, 0,
			    0, 0, -1,
			    1, 0, 0);
/*  tf::Matrix3x3 Ric(   0, 0, 1,
			    -1, 0, 0,
			    0, -1, 0);*/
	tf::Transform tfTiw ( tf::Matrix3x3( tfTcw.getRotation() ) * Ric, tfTcw.getOrigin() );
#else
    tf::Matrix3x3 Ric( 0,  0,  1,
			-1,  0,  0,
			0,  -1,  0);
      tf::Transform tfTiw ( Ric * tf::Matrix3x3( tfTcw.getRotation() ), Ric * tfTcw.getOrigin() );
#endif

    geometry_msgs::Transform gmTwi;
	tf::transformTFToMsg(tfTiw, gmTwi);

	geometry_msgs::Pose camera_pose_in_imu;
	camera_pose_in_imu.position.x = gmTwi.translation.x;
	camera_pose_in_imu.position.y = gmTwi.translation.y;
	camera_pose_in_imu.position.z = gmTwi.translation.z;
	camera_pose_in_imu.orientation = gmTwi.rotation;

    geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
	camera_odom_in_imu.header.frame_id = "map";
	camera_odom_in_imu.header.stamp = cv_ptrLeft->header.stamp;
	camera_odom_in_imu.pose.pose = camera_pose_in_imu;

	mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);

/*
    tf::Matrix3x3 Ric( 0,  0,  1,
			-1,  0,  0,
			0,  -1,  0);

    tf::Matrix3x3 Rbi( 0,  -1,  0,
			0,  0,  -1,
			1,  0,  0);

	tf::Transform tfTiw ( Ric * tf::Matrix3x3( tfTcw.getRotation() ) * Rbi, Ric * tfTcw.getOrigin() );
    geometry_msgs::Transform gmTwi;
	tf::transformTFToMsg(tfTiw, gmTwi);

	geometry_msgs::Pose camera_pose_in_imu;
	camera_pose_in_imu.position.x = gmTwi.translation.x;
	camera_pose_in_imu.position.y = gmTwi.translation.y;
	camera_pose_in_imu.position.z = gmTwi.translation.z;
	camera_pose_in_imu.orientation = gmTwi.rotation;

    geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
	camera_odom_in_imu.header.frame_id = "odom";
	camera_odom_in_imu.header.stamp = cv_ptrLeft->header.stamp;
	camera_odom_in_imu.pose.pose = camera_pose_in_imu;

	mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);
*/

// #ifdef FRAME_WITH_INFO_PUBLISH
    if (mpSLAM != NULL && mpSLAM->mpFrameDrawer != NULL) {
        cv::Mat fr_info_cv = mpSLAM->mpFrameDrawer->DrawFrame();
        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptrLeft->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = fr_info_cv; // Your cv::Mat
        mpFrameWithInfoPublisher.publish(out_msg.toImageMsg());
    }
// #endif
}


void ImageGrabber::GrabOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    /*
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
    msg->pose.pose.position.x,msg->pose.pose.position.y,
    msg->pose.pose.position.z); ROS_INFO("Orientation-> x: [%f], y: [%f], z:
    [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w); ROS_INFO("Vel->
    Linear: [%f], Angular: [%f]",
    msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    */

    // TODO
    mpSLAM->mpTracker->motion_model_.append(
        {msg->header.stamp.toSec(),
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z,
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z}
    );
}

bool ImageGrabber::HandlePauseRequest(std_srvs::SetBool::Request&  req,
                                      std_srvs::SetBool::Response& res) {
    paused      = req.data;
    res.success = true;
    res.message = paused ? "ORB3 Paused" : "ORB3 Resumed";
    return true;
}

