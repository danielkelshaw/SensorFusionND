
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    //
    /* INIT VARIABLES AND DATA STRUCTURES */
    //

    std::string filename = "performance_report.csv";
    std::ofstream outputFile(filename, ios::out);

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;  // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // vectors of detectors/descriptors/matchers/selectors to test
    vector<std::string> detectorTypeList   = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<std::string> descriptorTypeList = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    vector<std::string> matcherTypeList    = {"MAT_BF"};
    vector<std::string> selectorTypeList   = {"SEL_KNN"};

    // gnerate a vector of every combination to test
    vector<performanceStatistics> combinations;
    for (const auto& detector_type : detectorTypeList)
    {
        for (const auto& descriptor_type : descriptorTypeList)
            {
                for (const auto& matcher_type : matcherTypeList)
                {
                    for (const auto& selector_type : selectorTypeList)
                    {
                        performanceStatistics combo;
                        combo.detectorType   = detector_type;
                        combo.descriptorType = descriptor_type;
                        combo.matchingType   = matcher_type;
                        combo.selectorType   = selector_type;

                        for (int i =  0 ; i < imgEndIndex; i++)
                        {
                            combo.detectorTime[i]   = 0.0f;
                            combo.descriptorTime[i] = 0.0f;
                            combo.MatcherTime[i]    = 0.0f;
                            combo.numKeyPointsPerframe[i] = 0;
                            combo.numMatchedKeyPoints[i]  = 0;
                            combo.numKeyPointsPerROI[i]   = 0;
                        }

                        combinations.push_back(combo);

                    }
                }
            }       
    }

    for (auto& combo : combinations)
    {
        std::string detectorType = combo.detectorType;
        std::string descriptorType = combo.descriptorType;
        std::string matcherType = combo.matchingType;
        std::string selectorType = combo.selectorType;

        cout << endl << "****** " << detectorType << "--" << descriptorType << "--"<< matcherType << "--"<< selectorType << " *****" << endl;

        if ((descriptorType.compare("AKAZE") == 0 && detectorType.compare("AKAZE") != 0 ) || 
            ( descriptorType.compare("ORB") == 0 && detectorType.compare("SIFT") == 0 ) )
        {
            continue;
        }

        dataBuffer.clear();

        //
        /* MAIN LOOP OVER ALL IMAGES */
        //

        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
        {
            /* LOAD IMAGE INTO BUFFER */
            ptInfo performance = {0, 0.0f};
            cout << endl << "================ Image " << imgIndex << " ================" << endl;

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file 
            cv::Mat img = cv::imread(imgFullFilename);

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = img;
            dataBuffer.push_back(frame);

            cout << "#1 : LOAD IMAGE INTO BUFFER" << endl;

            //
            /* DETECT & CLASSIFY OBJECTS */
            //

            float confThreshold = 0.2;
            float nmsThreshold = 0.4;        
            detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                           yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

            cout << "#2 : DETECT & CLASSIFY OBJECTS" << endl;

            //
            /* CROP LIDAR POINTS */
            //

            // load 3D Lidar points from file
            string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
            std::vector<LidarPoint> lidarPoints;
            loadLidarFromFile(lidarPoints, lidarFullFilename);

            // remove Lidar points based on distance properties
            float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
        
            (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

            cout << "#3 : CROP LIDAR POINTS" << endl;

            //
            /* CLUSTER LIDAR POINT CLOUD */
            //

            // associate Lidar points with camera-based ROI
            float shrinkFactor = 0.15;
            clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

            // Visualize 3D objects
            bVis = false;
            if(bVis)
            {
                show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
            }
            bVis = false;

            cout << "#4 : CLUSTER LIDAR POINT CLOUD" << endl;
            
            //
            /* DETECT IMAGE KEYPOINTS */
            //

            // convert current image to grayscale
            cv::Mat imgGray;
            cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

            // extract 2D keypoints from current image
            vector<cv::KeyPoint>keypoints;
                
            if (detectorType.compare("SHITOMASI") == 0)
            {
                performance =  detKeypointsShiTomasi(keypoints, imgGray, false);
            }
            else if (detectorType.compare("HARRIS") == 0)
            {
                performance = detKeypointsHarris(keypoints, imgGray, false);
            }
            else
            {
                performance = detKeypointsModern(keypoints, imgGray, detectorType, false);
            }

            combo.numKeyPointsPerframe[imgIndex] = performance.numPoints;
            combo.detectorTime[imgIndex] = performance.elaspsedtime_ms;

            // optional : limit number of keypoints (helpful for debugging and learning)
            bool bLimitKpts = false;
            if (bLimitKpts)
            {
                int maxKeypoints = 50;

                if (detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                cout << " NOTE: Keypoints have been limited!" << endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (dataBuffer.end() - 1)->keypoints = keypoints;

            cout << "#5 : DETECT KEYPOINTS" << endl;

            //
            /* EXTRACT KEYPOINT DESCRIPTORS */
            //

            cv::Mat descriptors;
            performance = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
            combo.descriptorTime[imgIndex] = performance.elaspsedtime_ms;

            // push descriptors for current frame to end of data buffer
            (dataBuffer.end() - 1)->descriptors = descriptors;

            cout << "#6 : EXTRACT DESCRIPTORS" << endl;


            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {

                //
                /* MATCH KEYPOINT DESCRIPTORS */
                //

                vector<cv::DMatch> matches;
                string descriptorCategory = (descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY";

                performance = matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                               (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                                matches, descriptorCategory, matcherType, selectorType);

                combo.numMatchedKeyPoints[imgIndex] = performance.numPoints;
                combo.MatcherTime[imgIndex] = performance.elaspsedtime_ms;

                // store matches in current data frame
                (dataBuffer.end() - 1)->kptMatches = matches;

                cout << "#7 : MATCH KEYPOINT DESCRIPTORS" << endl;

                //
                /* TRACK 3D OBJECT BOUNDING BOXES */
                //

                map<int, int> bbBestMatches;
                matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1));

                // store matches in current data frame
                (dataBuffer.end()-1)->bbMatches = bbBestMatches;

                cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES" << endl;

                //
                /* COMPUTE TTC ON OBJECT IN FRONT */
                //

                // loop over all BB match pairs
                for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
                {
                    // find bounding boxes associates with current match
                    BoundingBox *prevBB, *currBB;
                    for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            currBB = &(*it2);
                        }
                    }

                    for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            prevBB = &(*it2);
                        }
                    }

                    // compute TTC for current match
                    if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 )
                    {
                        double ttcLidar; 
                        computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                        combo.ttcLidar[imgIndex] = ttcLidar;

                        double ttcCamera;
                        clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);                    
                        computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                        combo.ttcCamera[imgIndex] = ttcCamera;

                        bVis = false;
                        if (bVis)
                        {
                            cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                            showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                            cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                            
                            char str[200];
                            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                            putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                            string windowName = "Final Results : TTC";
                            cv::namedWindow(windowName, 4);
                            cv::imshow(windowName, visImg);
                            cout << "Press key to continue to next frame" << endl;
                            cv::waitKey(0);
                        }
                        bVis = false;

                    }
                }          
            }
        } 
    }

    //
    /* PRINT REPORT */
    //

    outputFile << "detector_type"
               << ","
               << "descriptor_type"
               << ","
               << "frame"
               << ","
               << "no_keypoints_frame"
               << ","
               << "detector_time_ms"
               << ","
               << "descriptor_time_ms"
               << ","
               << "no_matched_points"
               << ","
               << "matching_time_ms"
               << ","
               << "ttc_lidar_s"
               << ","
               << "ttc_camera_s"
               << std::endl;

    for (const auto& combo : combinations)
    {
        for (int i = 0; i < imgEndIndex; i++)
        {
            outputFile << combo.detectorType
                       << "," << combo.descriptorType
                       << "," << i
                       << "," << combo.numKeyPointsPerframe[i]
                       << "," << std::fixed << std::setprecision(8) << combo.detectorTime[i]
                       << "," << std::fixed << std::setprecision(8) << combo.descriptorTime[i]
                       << "," << combo.numMatchedKeyPoints[i]
                       << "," << std::fixed << std::setprecision(8) << combo.MatcherTime[i]
                       << "," << std::fixed << std::setprecision(8) << combo.ttcLidar[i]
                       << "," << std::fixed << std::setprecision(8) << combo.ttcCamera[i]
                       << endl;
        }

        outputFile << endl;
    }

    outputFile.close();

    return 0;
}
