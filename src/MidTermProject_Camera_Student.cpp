/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

// create an ofstream for the file output
std::string filename = "PerfReport.csv";
std::ofstream outputFile(filename, ios::out);

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    vector<std::string> detectorTypeList   = { "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};  
    vector<std::string> descriptorTypeList = { "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};                        
    vector<std::string> matcherTypeList    = { "MAT_BF"};
    vector<std::string> selectorTypeList   = { "SEL_KNN"};

    vector<perfStats> Combinations;
    for(const auto& a_detector_type : detectorTypeList )
    {
        for(const auto& a_descriptor_type : descriptorTypeList )
            {
                for(const auto& a_matcher_type : matcherTypeList )
                {
                    for(const auto& a_selector_type : selectorTypeList )
                    {
                        perfStats newCombo;
                        newCombo.detectorType   = a_detector_type;
                        newCombo.descriptorType = a_descriptor_type;
                        newCombo.matchingType   = a_matcher_type;
                        newCombo.selectorType   = a_selector_type;
                        for( int i=0 ; i < 10; i++)
                        {
                            newCombo.detectorTime[i] =0.0f;
                            newCombo.descriptorTime[i] =0.0f;
                            newCombo.MatcherTime[i]= 0.0f;
                            newCombo.numKeyPointsPerframe[i] = 0;
                            newCombo.numMatchedKeyPoints[i]  = 0;
                            newCombo.numKeyPointsPerROI[i]   = 0;
                        }
                        Combinations.push_back(newCombo);

                    }
                }
            }       
    }


    for( auto& a_combo : Combinations )
    {
        std::string detectorType   = a_combo.detectorType;
        std::string descriptorType = a_combo.descriptorType;
        std::string matcherType    = a_combo.matchingType;
        std::string selectorType   = a_combo.selectorType;

        cout << endl << "****** " << detectorType << "--" << descriptorType << "--"<< matcherType << "--"<< selectorType << " *****" << endl;

        if ( ( descriptorType.compare("AKAZE") == 0 && detectorType.compare("AKAZE") != 0 ) 
                || ( descriptorType.compare("ORB") == 0 && detectorType.compare("SIFT") == 0 ) )
                continue;


        /* MAIN LOOP OVER ALL IMAGES */
        dataBuffer.clear();
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
        {
            /* LOAD IMAGE INTO BUFFER */
            returnInfo perf = {0, 0.0f};

            cout << endl <<"================ Image " << imgIndex << "================" << endl;

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file and convert to grayscale
            cv::Mat img, imgGray;
            img = cv::imread(imgFullFilename);
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

            //// STUDENT ASSIGNMENT
            //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = imgGray;
            if(dataBuffer.size() < dataBufferSize)
            {
                dataBuffer.push_back(frame);
                cout << "Pushing new image to the ring buffer" <<  endl;
            }
            else
            {
                dataBuffer.erase(dataBuffer.begin());
                dataBuffer.push_back(frame);
                cout << "Replacing new image to the ring buffer" << endl;
            } 
            

            //// EOF STUDENT ASSIGNMENT
            cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

            /* DETECT IMAGE KEYPOINTS */

            // extract 2D keypoints from current image
            vector<cv::KeyPoint>keypoints; // create empty feature list for current image
            
            //// STUDENT ASSIGNMENT
            //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
            //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
            if (detectorType.compare("SHITOMASI") == 0)
            {
                perf =  detKeypointsShiTomasi(keypoints, imgGray, false);
            }
            else if (detectorType.compare("HARRIS") == 0)
            {
                perf = detKeypointsHarris(keypoints, imgGray, false);
            }
            else
            {
                perf = detKeypointsModern(keypoints, imgGray, detectorType, false);
            }
            //// EOF STUDENT ASSIGNMENT
            a_combo.numKeyPointsPerframe[imgIndex] = perf.numPoints;
            a_combo.detectorTime[imgIndex] = perf.elaspsedtime_ms;

            //// STUDENT ASSIGNMENT
            //// TASK MP.3 -> only keep keypoints on the preceding vehicle

            // only keep keypoints on the preceding vehicle
            bool bFocusOnVehicle = true;
            cv::Rect vehicleRect(535, 180, 180, 150);
            vector<cv::KeyPoint> keypoints_roi;
            if (bFocusOnVehicle)
            {
                for (auto itr = keypoints.begin(); itr != keypoints.end(); ++itr)
                {
                    if (vehicleRect.contains((*itr).pt))
                    {
                        cv::KeyPoint newKeyPoint;
                        keypoints_roi.push_back(*itr);
                    }
                }
                keypoints = keypoints_roi;
                a_combo.numKeyPointsPerROI[imgIndex] = keypoints.size();
                cout << "# of keypoints in ROI n =  " << keypoints.size() << endl;
            }

            //// EOF STUDENT ASSIGNMENT

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
            cout << "#2 : DETECT KEYPOINTS done" << endl << endl;

            /* EXTRACT KEYPOINT DESCRIPTORS */

            //// STUDENT ASSIGNMENT
            //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
            //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
            cv::Mat descriptors;
            perf = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
            a_combo.descriptorTime[imgIndex] = perf.elaspsedtime_ms;
            //// EOF STUDENT ASSIGNMENT

            // push descriptors for current frame to end of data buffer
            (dataBuffer.end() - 1)->descriptors = descriptors;

            cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {
                /* MATCH KEYPOINT DESCRIPTORS */
                vector<cv::DMatch>  matches;
                //// STUDENT ASSIGNMENT
                //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                string descriptorStyle = ( descriptorType.compare("SIFT")==0 ) ? "DES_HOG" : "DES_BINARY";           

                perf = matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                matches, descriptorStyle, matcherType, selectorType);

                a_combo.numMatchedKeyPoints[imgIndex] = perf.numPoints;
                a_combo.MatcherTime[imgIndex] = perf.elaspsedtime_ms;
                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                (dataBuffer.end() - 1)->kptMatches = matches;

                cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                // visualize matches between current and previous image
                bVis = false;
                if (bVis)
                {
                    cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                    cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                    (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                    matches, matchImg,
                                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                                    vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                    string windowName = "Matching keypoints between two camera images";
                    cv::namedWindow(windowName, 7);
                    cv::imshow(windowName, matchImg);
                    cout << "Press key to continue to next image" << endl;
                    cv::waitKey(0); // wait for key to be pressed
                }
                bVis = false;
                } // end of if size  > 1
        } // eof loop over all images
    } // eof loop over all combinations

    //Print Report
    // create and open the .csv file
    outputFile << "Detector Type"
               << ","
               << "Descriptor Type"
               << ","
               << "Frame#"
               << ","
               << "#KeyPointsPerFrame"
               << ","
               << "#KeyPointsPerROI"
               << ","
               << "DetectorTime(ms)"
               << ","
               << "DescriptorTime(ms)"
               << ","
               << "#MatchedPoints"
               << ","
               << "MatchingTime(ms))"
               << std::endl;

    for( const auto& a_combo : Combinations )
    {
        for( int i= 0 ; i < 10; i++)
        {
            outputFile << a_combo.detectorType
                       << "," << a_combo.descriptorType
                       << "," << i
                       << "," << a_combo.numKeyPointsPerframe[i]
                       << "," << a_combo.numKeyPointsPerROI[i]
                       << "," << std::fixed << std::setprecision(8) << a_combo.detectorTime[i]
                       << "," << std::fixed << std::setprecision(8) << a_combo.descriptorTime[i]
                       << "," << a_combo.numMatchedKeyPoints[i]
                       << "," << std::fixed << std::setprecision(8) << a_combo.MatcherTime[i]
                       << endl;
        }
        outputFile << endl;
    }
    outputFile.close();
    return 0;
}
