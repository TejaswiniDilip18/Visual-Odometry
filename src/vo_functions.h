#include <iostream>
#include <string>
#include <vector>
#include <ctype.h>
#include <fstream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "LatLong-UTMconversion.h"  // Include UTM conversion functions
#include "constants.h" // Include constants header with ellipsoid definitions

using namespace std;

// Function to detect features in an image using Shi-Tomasi Corner Detector (Good Features to Track)
void feature_detection(const cv::Mat& img, std::vector<cv::Point2f>& points) {
    int max_corners = 2000;
    double quality_level = 0.01;
    double min_distance = 15;

    cv::goodFeaturesToTrack(img, points, max_corners, quality_level, min_distance);

    if (points.size() < 1000) {
        // If we don't have enough features, lower the quality threshold
        quality_level /= 2;
        cv::goodFeaturesToTrack(img, points, max_corners, quality_level, min_distance);
    }
}

// Function to track features using optical flow 
void feature_tracking(cv::Mat img1, cv::Mat img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, vector<uchar>& status){

    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    vector<float> err;
    cv::Size winSize = cv::Size(21, 21);

    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
        {  cv::Point2f pt = points2.at(i- indexCorrection);
            if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
                if((pt.x<0)||(pt.y<0))	{
                    status.at(i) = 0;
                }
                points1.erase (points1.begin() + (i - indexCorrection));
                points2.erase (points2.begin() + (i - indexCorrection));
                indexCorrection++;
            }
        }
}

// Function to get the true pose from the ground truth data for comparison
void truePose(int frame_id, double& x, double& y, double& z, std::string true_pose) {
    std::string line;
    int i = 0;
    std::ifstream myfile(true_pose);
    if (myfile.is_open()) {
        while (getline(myfile, line) && i < frame_id) {
            std::istringstream in(line);
            for (int j = 0; j < 12; j++) {
                in >> z;
                if (j == 3) x = z; // x
                if (j == 7) y = z; // y
            }
            i++;
        }
        myfile.close();
    } else {
        std::cerr << "Unable to open ground truth file" << std::endl;
    }
}

// Function to get the absolute scale of the trajectory from GPS data
double getAbsoluteScale(int frame_id, std::string oxts_data) {
    std::string root_path = oxts_data;
    char filename[150];
    char filename_prev[150];

    std::sprintf(filename, "%s/%010d.txt", root_path.c_str(), frame_id);
    std::sprintf(filename_prev, "%s/%010d.txt", root_path.c_str(), frame_id - 1);

    std::ifstream myfile(filename);
    std::ifstream myfile_prev(filename_prev);

    std::vector<double> lats, lons, alts;
    std::vector<double> utmNorthings, utmEastings;

    // Read GPS Data
    if (myfile.is_open() && myfile_prev.is_open()) {
        std::string line;
        while (getline(myfile, line)) {
            std::istringstream in(line);
            double lat, lon, alt;
            for (int j = 0; j < 3; j++) {
                in >> alt;
                if (j == 0) lat = alt;
                if (j == 1) lon = alt;
            }
            lats.push_back(lat);  // Latitude
            lons.push_back(lon);  // Longitude
            alts.push_back(alt);  // Altitude
        }

        myfile.close();

        while (getline(myfile_prev, line)) {
            std::istringstream in(line);
            double lat, lon, alt;
            for (int j = 0; j < 3; j++) {
                in >> alt;
                if (j == 0) lat = alt; 
                if (j == 1) lon = alt; 
            }
            lats.push_back(lat); // Latitude
            lons.push_back(lon); // Longitude
            alts.push_back(alt); // Altitude
        }

        myfile_prev.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
        return -1; // Use -1 or some other error code for failure
    }

    // Convert GPS coordinates to UTM
    int referenceEllipsoid = 23; // WGS-84
    char utmZone[4];

    for (size_t i = 0; i < lats.size(); ++i) {
        double utmNorthing, utmEasting;
        LLtoUTM(referenceEllipsoid, lats[i], lons[i], utmNorthing, utmEasting, utmZone);
        utmNorthings.push_back(utmNorthing);
        utmEastings.push_back(utmEasting);
    }

    // Calculate the distance between the first and last UTM coordinates
    double dx = utmEastings.back() - utmEastings.front();
    double dy = utmNorthings.back() - utmNorthings.front(); // Change in northing
    double distance = std::sqrt(dx * dx + dy * dy); // Euclidean distance

    //Calculate scale based on distance and altitude change
    double dz = alts.back() - alts.front(); // Change in altitude
    double gps_scale = std::sqrt(dx * dx + dy * dy + dz * dz);

    return gps_scale;
}