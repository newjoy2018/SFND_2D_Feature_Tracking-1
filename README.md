# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


## Rubric

1. Data Buffer Optimization: Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). 

   

   > Implemented a ring bufgfer with dataBufferSize = 2. Push the current image into data frame buffer as long as ring buffer size is less that the max buffer size(dataBufferSize), 
     If ring buffer size is greater than or equal to dataBufferSize, erase the oldest element and repalce it with new image.

   

2. Keypoint Detection: Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

   

   >  Implemented 3 different functions: 
   >	- detKeypointsShiTomasi (Shi-Thomasi, Good Features to Track),
		- detKeypointsHarris(Harris corner), 
		- detKeypointsModern(FAST, BRISK, ORB, AKAZE, and SIFT). 
		- In each function, everal parameters(block size, minimal distance, threshold...) were set up as required by a partiucalr detector
		- OpenCV build-in detector class will be initialized with these parameters and scan the whole image to detect key-points. 
		- Also logged #keypoints deteceted and time spent in detecting them for performance evaluation.
   

3. Keypoint Removal: Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

   

   > Extracted the keypoints in hte ROI defined by Vechicle Selection reactangle.

   

4. Keypoint Descriptors: Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

   

   > Similar to step 2, implement a function to select a descriptor type based on the input string. 
     Used OpenCV build-in descriptors (BRIEF, ORB, FREAK, AKAZE and SIFT) class with default parameters to uniquely identify keypoints.
	 Logged the descriptor extraction time for performance evaluation.

   

5. Descriptor Matching: Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

   

   >Matching methods Brute-force matcher(MAT_BF) or FLANN matcher(MAT_FLANN) can be selected based on a string.
    Also implemented KNN selection with default of 2 nearest neighors. Filtered out keypoints for matching based on Descriptr Distance Ratio of 0.8 
       

6. Descriptor Distance Ratio: Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

   

   > Nearest neighbor (best match) and K nearest neighbors (default k=2) selectors are implemented. For KNN, filtered out keypoints for best matching based on Descriptr Distance Ratio of 0.8.

   
#### 7. Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

| Detector | Number of detected Keypoints on the preceding vehicle for total of 10 images |
| --- | --- |
| **SHITOMASI** | 1179 |
| **HARRIS** | 248 |
| **FAST** | 1491 |
| **BRISK** | 2762 |
| **ORB** | 1161 |
| **AKAZE** | 1670 |
| **SIFT** | 1386 |

#### 8. The number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| **SHITOMASI** | 767 |944|908|768|N/A|927|
| **HARRIS** | 142|173 |162|144|N/A|163|
| **FAST** | 899 |1099|1071|878|N/A|1046|
| **BRISK** | 1570 |1704|1514|1524|N/A|1646|
| **ORB** | 751 |545|763|420|N/A|763|
| **AKAZE** | 1215 |1266|1182|1187|1259|1270|
| **SIFT** | 592 |702|Out of Memory|593|N/A|800|

#### 9. Average Processing Time(DETECTOR + DESCRIPTOR)(ms) on all 10 images for each detector/descriptor combination

| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| **SHITOMASI** | 18.34 |9.92|10.65|38.12|N/A| 27.61|
| **HARRIS** | 10.14|9.78 |10.07|37.40| N/A| 25.09|
| **FAST** | 2.00|1.50|1.97|28.60|N/A|15.33|
| **BRISK** | 30.9 |29.84|34.16|57.69|N/A|52.64|
| **ORB** | 6.59 |5.92|9.11|33.64|N/A|30.43|
| **AKAZE** | 45.54 |44.72|43.77|68.42|90.17|55.42|
| **SIFT** | 94.30 |104.03|Out of Memory|125.67|N/A|194.89|

Top 3 detector/descriptor combinations based on more keypoints matches in less amount of time:

DETECTOR/DESCRIPTOR  | NUMBER OF KEYPOINTS | TIME
-------------------- | --------------------| --------
FAST+BRIEF           | 1099 keypoints    | 1.50 ms 
FAST+ORB             | 1071 keypoints    | 1.97 ms 
FAST+BRISK           | 899 keypoints     | 2.00 ms 