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

#include<opencv2/core/core.hpp>

#include<System.h>

#include<boost/program_options.hpp>
#include<boost/filesystem.hpp>

using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadIMU(const string &strImuPath, vector<ORB_SLAM3::IMU::Point> &vImus);

void getImuMeasurements(const vector<ORB_SLAM3::IMU::Point> &vImus, double t1, double t2,
                        vector<ORB_SLAM3::IMU::Point> &vImuMeas);

int main(int argc, char **argv)
{
    // if(argc != 5)
    // {
    //     cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
    //     return 1;
    // }

    std::string path_to_vocabulary;
    std::string path_to_settings;
    std::string path_to_sequence;
    std::string association_filename;
    std::string imu_filename;
    std::string dataset;
    std::string output_dir;
    float speed = 1.0;
    bool use_viewer = true;
    po::options_description desc;
    desc.add_options()("help,h", "show this message")(
        "path_to_vocabulary,v",
        po::value<std::string>(&path_to_vocabulary)->default_value(""),
        "path_to_vocabulary")(
        "path_to_settings,s",
        po::value<std::string>(&path_to_settings)->default_value(""),
        "path_to_settings")(
        "path_to_sequence,q",
        po::value<std::string>(&path_to_sequence)->default_value(""),
        "path_to_sequence")(
        "association_filename,a",
        po::value<std::string>(&association_filename)->default_value(""),
        "association filename")(
        "dataset,t", po::value<std::string>(&dataset)->default_value("dummy"),
        "dataset name (default: dummy)")(
        "output_dir,o",
        po::value<std::string>(&output_dir)->default_value("/tmp/orb3_rgbd/"),
        "output_dir (default: /tmp)")(
        "speed,d", po::value<float>(&speed)->default_value(1.0),
        "playing speed")(
        "use_viewer,w", po::value<bool>(&use_viewer)->default_value(true),
        "use viewer")("imu_filename,i",
                      po::value<std::string>(&imu_filename)->default_value(""),
                      "imu filename, use imu if provided (default: empty)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return -1;
    }
    if (path_to_sequence.empty() || path_to_vocabulary.empty() || association_filename.empty())
    {
        std::cout << "wrong path_to_sequence or path_to_vocabulary or association_filename... " << std::endl;
        return -1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    vector<ORB_SLAM3::IMU::Point> vImus;
    // string strAssociationFilename = string(argv[4]);
    LoadImages(association_filename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    if (!imu_filename.empty())
    {
        LoadIMU(imu_filename, vImus);
        std::cout << "Loaded " << vImus.size() << " imus" << std::endl;
    }

    // Create output dir.
    {
        fs::path dir_path(output_dir);
        if (!fs::exists(dir_path)) {
            fs::create_directory(dir_path);
        }
    }
    std::string output_dir_prefix = output_dir + "/" + dataset;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto mode = vImus.size() > 0 ? ORB_SLAM3::System::IMU_RGBD : ORB_SLAM3::System::RGBD;
    ORB_SLAM3::System SLAM(path_to_vocabulary, path_to_settings,mode,use_viewer);
    SLAM.mpTracker->SetRealTimeFileStream(output_dir_prefix + "_AllFrameTrajectory.txt");
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    // vTimesTrack.resize(nImages);
    vTimesTrack.reserve(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    int proccIm = 0;
    int imCount = nImages + 1;
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(path_to_sequence+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(path_to_sequence+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        if (ni > 0)
        {
            getImuMeasurements(vImus, vTimestamps[ni-1], tframe, vImuMeas);
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe, vImuMeas);
        proccIm++;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // vTimesTrack[ni]=ttrack;
        vTimesTrack.emplace_back(ttrack);

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        // if(ttrack<T)
        //     usleep((T-ttrack)*1e6);

        // consider speed
        T = T / speed;
        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
        else // catch up with images
        {
            while (ni < nImages - 1 && 
                    (vTimestamps[ni+1] - tframe) / speed < ttrack)
            {
                ni++;
            }
        }

        if (proccIm > imCount)
        {
            break;
        }
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    proccIm = vTimesTrack.size();
    for(int ni=0; ni<proccIm; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[proccIm/2] << endl;
    cout << "mean tracking time: " << totaltime/proccIm << endl;

    {
        // SLAM.SaveTrajectoryTUM(prefix + "_AllFrameTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryTUM(output_dir_prefix + "_KeyFrameTrajectory.txt");
        SLAM.SaveTrackingLog(output_dir_prefix + "_Log_Tracking.txt" );
        SLAM.SaveMappingLog(output_dir_prefix + "_Log_Mapping.txt"); 
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<ORB_SLAM3::IMU::Point>& vImus)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vImus.reserve(5000);
    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t, wx, wy, wz, ax, ay, az;
            ss >> t >> wx >> wy >> wz >> ax >> ay >> az;
            vImus.emplace_back(ax, ay, az, wx, wy, wz, t);
        }
    }

    std::sort(
        vImus.begin(), vImus.end(),
        [](const ORB_SLAM3::IMU::Point &left,
           const ORB_SLAM3::IMU::Point &right) { return left.t < right.t; });
}

void getImuMeasurements(const vector<ORB_SLAM3::IMU::Point> &vImus, double t1, double t2,
                   vector<ORB_SLAM3::IMU::Point> &vImuMeas) {
    vImuMeas.clear();
    std::copy_if(vImus.cbegin(), vImus.cend(), std::back_inserter(vImuMeas),
        [t1, t2](const ORB_SLAM3::IMU::Point& elem) {
            return elem.t >= t1 && elem.t <= t2;
        }
    );
}