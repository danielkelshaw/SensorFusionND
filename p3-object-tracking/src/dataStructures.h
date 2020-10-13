
#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <map>
#include <opencv2/core.hpp>

struct LidarPoint {

    double x,y,z,r; // x,y,z in [m], r is point reflectivity

};

struct BoundingBox {
    
    int boxID;   // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs
    
    cv::Rect roi;      // 2D region-of-interest in image coordinates
    int classID;       // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches;  // keypoint matches enclosed by 2D roi

};

struct DataFrame { 

    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors;                 // keypoint descriptors
    std::vector<cv::DMatch> kptMatches;  // keypoint matches between previous and current frame
    std::vector<LidarPoint> lidarPoints;

    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int,int> bbMatches;            // bounding box matches between previous and current frame

};

struct performanceStatistics {

    std::string detectorType;
    std::string descriptorType;
    std::string matchingType;
    std::string selectorType;

    int numKeyPointsPerframe[20];
    int numKeyPointsPerROI[20];
    int numMatchedKeyPoints[20];

    double detectorTime[20];
    double descriptorTime[20];
    double MatcherTime[20];

    double ttcCamera[20];
    double ttcLidar[20];

};

struct ptInfo {

        int numPoints;
        double elaspsedtime_ms;

};

#endif /* dataStructures_h */
