# Sensor Fusion: 3D Object Tracking

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Download *yolov3.weights*: ```wget https://pjreddie.com/media/files/yolov3.weights```
3. Make a build directory in the top level project directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./3D_object_tracking`.

## Rubric

#### **FP1: Match 3D Objects**

Implemented the method ```matchBoundingBoxes()``` which takes the current/previous data frames as inputs and outputs the ids of the matched regions of interest. Matches must be the ones with the highest number of keypoint correspondences.

#### **FP2: Compute Lidar TTC**

Implemented the method ```computeTTCLidar()``` which computes the TTC for matched 3D objects using only lidat measurments from the matched bounding boxes between the current/previous frames.

In order to reduce the risk of outliers only the lidar points within the ego lane were considered and the mean distance was taken in order to produce a stable output.

#### **FP3: Associate Keypoints with Bounding Boxes**

Implemented the method: ```clusterKptMatchesWithROI()``` which associates keypoints with the bounding box that encloses them. All matches between current/previous frame are pushed to a vector within the respective bounding box.

#### **FP4: Compute Camera TTC**

Implemented the method: ```computeTTCCamera()``` which examines the distance ratios between keypoint matches in the current/previous frames - this ratio is then used to compute the TTC.

#### **FP5: Performance Evaluation**

There were a few cases where there were no lidar points within a few bounding boxes. I adjusted the ShrinkFactpr in order to achieve a more reliable Lidar-based TTC. This could have occured due to overlapping boxes etc.

#### **FP6: Performance Evaluation**

Upon running several detector/descriptor combinations I found that the HARRIS/ORB detectors were not giving reliable results - this can be seen within the performance report generated: ```results/performance_report.csv```.
<br>
Based on some rudimentary analysis of the total processing time as well as the mean absolute difference in TTC estimates I have found that some of the best combinations of detectors/descriptors are:
<br>
- AKAZE/BRISK
- FAST/FREAK
- SHITOMASI/BRIEF
<br>

**Mean Difference in TTC Estimate (s):**

| Detector\Descriptor | AKAZE  | BRIEF  | BRISK   | FREAK   | ORB    | SIFT    |
| ------------------- | ------ | ------ | ------- | ------- | ------ | ------- |
| **AKAZE**           | 1.527  | 1.231  | 1.090   | 1.217   | 1.269  | 1.121   |
| **BRISK**           | NaN    | 3.076  | 3.325   | 3.0538  | 3.590  | 3.024   |
| **FAST**            | NaN    | 2.033  | 2.166   | 1.866   | 1.991  | 1.644   |
| **SHITOMASI**       | NaN    | 1.775  | 1.486   | 1.590   | 1.382  | 1.427   |
| **SIFT**            | NaN    | 1.577  | 1.676   | 1.376   | NaN    | 1.402   |


**Mean Total Processing Time (ms):**

| Detector\Descriptor | AKAZE  | BRIEF  | BRISK   | FREAK   | ORB    | SIFT    |
| ------------------- | ------ | ------ | ------- | ------- | ------ | ------- |
| **AKAZE**           | 99.457 | 62.435 | 72.160  | 85.870  | 65.580 | 93.472  |
| **BRISK**           | NaN    | 69.482 | 89.422  | 87.944  | 73.900 | 179.619 |
| **FAST**            | NaN    | 13.679 | 25.206  | 34.768  | 13.775 | 53.845  |
| **SHITOMASI**       | NaN    | 19.321 | 31.450  | 41.528  | 19.618 | 46.532  |
| **SIFT**            | NaN    | 91.410 | 105.166 | 112.653 | NaN    | 153.901 |




