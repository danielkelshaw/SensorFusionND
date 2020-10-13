# Udacity: Sensor Fusion Nanodegree

### **Projects:**

- [x] [**P1:** Lidar Obstacle Detection](https://github.com/danielkelshaw/SensorFusionND/tree/main/p1-lidar-obstacle-detection)

    This project involved processing point cloud data using the `C++` library, `PCL`. The data was first segemented using a linear Ransac model
    to determine which parts were part of the road, and which were not. KD-Trees were then used to conduct nearest neigbour search for clustering.
    A bounding box could then be drawn around the points - showing the location of vehicles.
    <br>

- [x] [**P2:** Feature Tracking](https://github.com/danielkelshaw/SensorFusionND/tree/main/p2-feature-tracking)

    The `OpenCV` library was used to conduct 2D feature tracking using a variety of keypoint detectors and descriptors. The descriptors were used
    to conduct keypoint matching from one video frame to the next - ultimately this could be used to calculate the time-to-collision for an
    autonomous vehicle.
    <br>

- [x] [**P3:** Object Tracking](https://github.com/danielkelshaw/SensorFusionND/tree/main/p3-object-tracking)

    The object tracking project utilised lidar data in conjunction with the 2D feature tracking in order to match the bounding boxes for vehicles
    and provide a much more representative estimate of the time-to-collision.
    <br>

- [x] [**P4:** Radar Target Detection](https://github.com/danielkelshaw/SensorFusionND/tree/main/p4-radar-target-detection)

    `MATLAB` code was developed to detect targets using a FMCW radar module. A FFT was applied to the radar data in order to produce a Range-Doppler
    estimation - CFAR was then applied to remove noise and gain an accurate signal. Once a clean Range-Doppler map had been acquired then clustering
    could be applied to track an individual object.
    <br>

- [x] [**P5:** Unscented Kalman Filter](https://github.com/danielkelshaw/SensorFusionND/tree/main/p5-unscented-kalman-filter)

    The aim of this project was to combine the acquired sensor data for use in a unified estimation of the state of a vehicle. An Unscented Kalman
    Filter was developed which took lidar and radar readings in order to simultaneously predict the state of multiple vehicles along a simulated
    road.
    <br>

###### Author: Daniel Kelshaw

