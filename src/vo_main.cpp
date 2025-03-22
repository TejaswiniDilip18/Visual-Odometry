#include <iostream>
#include <string>
#include <vector>
#include <ctype.h>
#include <algorithm>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "vo_functions.h"
#include <yaml-cpp/yaml.h>

using namespace std;

// Define constants
#define MIN_NUM_FEAT 2000

int main(){
    // values from KITTI's calib files
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);

    char text[100];
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 1;  
    cv::Point textOrg(10, 50);

    // Load YAML config file
    YAML::Node config = YAML::LoadFile("../config.yaml");

    // Extract paths
    string folder_path = config["dataset_path"].as<string>();
    string oxts_data = config["oxts_data_path"].as<string>();
    string true_pose = config["true_pose_path"].as<string>();

    char filename1[200];
    char filename2[200];

    sprintf(filename1, "%s/image_02/data/%010d.png", folder_path.c_str(), 0); // first frame
    sprintf(filename2, "%s/image_02/data/%010d.png", folder_path.c_str(), 1); // second frame

    // read first two frames
    cv::Mat img1 = cv::imread(filename1);
    cv::Mat img2 = cv::imread(filename2);

    if(!img1.data || !img2.data){
        cout<<"Error reading image files..."<<endl;
        return -1;
    }

    cv::Mat img1_gray, img2_gray;

    // conver images to grayscale
    cv::cvtColor(img1, img1_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, img2_gray, cv::COLOR_BGR2GRAY);

    // feature detection and tracking
    vector<cv::Point2f> points1, points2; 
    vector<uchar> status;
    feature_detection(img1_gray, points1);
    feature_tracking(img1_gray, img2_gray, points1, points2, status);

    // recover essential matrix and pose
    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, points2, points1, R, t, focal, pp, mask);

    cv::Mat R_f = R.clone();
    cv::Mat t_f = t.clone();

    cv::Mat prevImage = img2_gray;
    cv::Mat currImage;
    vector<cv::Point2f> prevFeatures = points2;
    vector<cv::Point2f> currFeatures;
    char filename[200];

    clock_t begin = clock();

    // Create windows for display
    cv::namedWindow( "Road facing camera", cv::WINDOW_AUTOSIZE );
    cv::namedWindow( "Trajectory", cv::WINDOW_AUTOSIZE );

    cv::Mat traj = cv::Mat::zeros(820, 950, CV_8UC3);

    std::vector<Pose> estimated_trajectory;
    std::vector<Pose> ground_truth_trajectory;

    int numFrame = 2;
    while(true){
        sprintf(filename, "%s/image_02/data/%010d.png", folder_path.c_str(), numFrame); // image path
        cv::Mat currImage_c = cv::imread(filename);
        if(currImage_c.empty()){
            break;
        }
        cv::cvtColor(currImage_c, currImage, cv::COLOR_BGR2GRAY);
        
        vector<uchar> status;
        feature_tracking(prevImage, currImage, prevFeatures, currFeatures, status);

        E = findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

        double gps_distance = getAbsoluteScale(numFrame, oxts_data);
        double scale = gps_distance / cv::norm(t);

        if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
            t_f = t_f + scale*(R_f*t);
            R_f = R*R_f;
        }
        else {
            // cout << "Scale calculation failed..." << endl;
        }

        estimated_trajectory.emplace_back(t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));

        // redetection if the number of features is below the threshold
        if(prevFeatures.size() < MIN_NUM_FEAT){
            feature_detection(prevImage, prevFeatures);
            feature_tracking(prevImage, currImage, prevFeatures, currFeatures, status);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        // update and draw trajectory 
        int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(2)) + 100;
        cv::circle(traj, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);

        //update and draw ground truth trajectory
        double x_true, y_true, z_true;
        truePose(numFrame, x_true, y_true, z_true, true_pose);
        int x_true_int = int(x_true) + 300;
        int y_true_int = int(z_true) + 100;
        cv::circle(traj, cv::Point(x_true_int, y_true_int), 1, CV_RGB(0, 255, 0), 1.5);

        ground_truth_trajectory.emplace_back(x_true, y_true, z_true);

        // Display coordinates
        cv::rectangle(traj, cv::Point(10, 30), cv::Point(800, 70), CV_RGB(0, 0, 0), cv::FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

        // Legend for estimated trajectory
        cv::rectangle(traj, cv::Point(650, 700), cv::Point(850, 750), CV_RGB(0, 0, 0), cv::FILLED);
        cv::circle(traj, cv::Point(700, 725), 5, CV_RGB(255, 0, 0), 5);
        cv::putText(traj, "Estimated Trajectory", cv::Point(725, 730), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness, 8);

        // Legend for ground truth
        cv::rectangle(traj, cv::Point(650, 750), cv::Point(850, 800), CV_RGB(0, 0, 0), cv::FILLED);
        cv::circle(traj, cv::Point(700, 775), 5, CV_RGB(0, 255, 0), 5);
        cv::putText(traj, "Ground Truth", cv::Point(725, 780), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness, 8);

        cv::imshow( "Road facing camera", currImage_c );
        cv::imshow( "Trajectory", traj );

        cv::waitKey(1);
        numFrame++;
    }

    // Save the trajectory as a PNG file
    cv::imwrite("../trajectory.png", traj);

    double ate = calculateATE(estimated_trajectory, ground_truth_trajectory);
    std::cout << "Absolute Trajectory Error: " << ate << " meters" << std::endl;

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Total time taken: " << elapsed_secs << "s" << endl;

    return 0;
}