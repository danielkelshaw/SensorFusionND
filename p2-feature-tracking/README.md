# **SFND: 2D Feature Tracking**

<img src="images/keypoints.png" width="820" height="248" />

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

### **Rubric**

#### **MP1: Data Buffer Optimisation**
Implemented a ring buffer with ```dataBufferSize=2```.
<br>
Push the current image into the buffer as long as the buffer size is less than ```dataBufferSize```.
<br>
If the ring buffer size is greater than or equal to ```dataBufferSize``` then erase the oldest element.

#### **MP2: Keypoint Detection**
Implemented detectors [HARRIS, FAST, BRISK, ORB, AKAZE, SIFT] through three functions:

```detKeypointsShiTomasi```
<br>
```detKeypointsHarris```
<br>
```detKeypointsModern```

The number of keypoints detected as well as the detection time were stored for performance evaluation.

#### **MP3: Keypoint Removal**

Removed all keypoints outside of a pre-defined rectangle.
<br>
Used only the points within the ROI for firther processing.

#### **MP4: Keypoint Descriptors**

Implemented descriptors [BRIEF, ORB, FREAK, AKAZE, SIFT]
<br>
The descriptor extraction time was logged for performance evaluation.

#### **MP5: Descriptor Matching**

Implemented FLANN matching as well as K-Nearest Neighbour selection.
<br>
Matching methods [MAT_BF, FLANN] can be selected based on an input string.

#### **MP6: Descriptor Distance Ratio**

For KNN, filtered keypoints for matching based on a minimum distance ratio of 0.8.

#### **MP7: Number of Keypoints on Preceding Vehicle**

| Detector      | Number of Keypoints in ROI |
| ------------- | -------------------------- |
| **AKAZE**     | 10020                      |
| **BRISK**     | 13810                      |
| **FAST**      | 7455                       |
| **HARRIS**    | 1240                       |
| **ORB**       | 5805                       |
| **SHITOMASI** | 5895                       |
| **SIFT**      | 5544                       |

#### **MP8: Number of Matched Keypoints**

| Detector\Descriptor | AKAZE | BRIEF | BRISK | FREAK | ORB  | SIFT |
| ------------------- | ----- | ----- | ----- | ----- | ---- | ---  |
| **AKAZE**           | 11259 | 1266  | 1215  | 1187  | 1186 | 1270 |
| **BRISK**           | NaN   | 1704  | 1570  | 1524  | 1510 | 1646 |
| **FAST**            | NaN   | 1099  | 899   | 878   | 1081 | 1046 |
| **HARRIS**          | NaN   | 173   | 142   | 144   | 160  | 163  |
| **ORB**             | NaN   | 545   | 751   | 420   | 761  | 763  |
| **SHITOMASI**       | NaN   | 944   | 767   | 768   | 907  | 927  |
| **SIFT**            | NaN   | 702   | 592   | 593   | NaN  | 800  |


#### **MP9: Average Processing Time**

| Detector\Descriptor | AKAZE   | BRIEF    | BRISK   | FREAK   | ORB     | SIFT    |
| ------------------- | ------- | -------- |-------- | ------- | ------- | ------- |
| **AKAZE**           | 732.308 | 421.137  | 447.541 | 627.814 | 461.105 | 585.463 |
| **BRISK**           | NaN     | 311.651  | 335.640 | 513.420 | 366.949 | 543.740 |
| **FAST**            | NaN     | 11.693   | 20.401  | 178.931 | 25.234  | 98.872  |
| **HARRIS**          | NaN     | 91.586   | 94.196  | 253.002 | 107.735 | 188.229 |
| **ORB**             | NaN     | 50.105   | 80.661  | 215.859 | 107.910 | 249.276 |
| **SHITOMASI**       | NaN     | 65.252   | 93.345  | 231.792 | 81.324  | 172.291 |
| **SIFT**            | NaN     | 613.213  | 636.415 | 769.844 | NaN     | 977.216 |


#### **Top Three Detector/Descriptor Combinations:**

Detector/Descriptor  | Number of Matched Keypoints | Processing Time |
-------------------- | --------------------------- | --------------- |
FAST+BRIEF           | 1099 keypoints              | 11.69 ms        |
FAST+BRISK           | 899 keypoints               | 20.40 ms        |
FAST+ORB             | 1081 keypoints              | 25.23 ms        |
