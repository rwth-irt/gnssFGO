# gnssFGO: an online and time-centric factor graph optimization for GNSS/Multi-sensor vehicle localization

[![IMAGE ALT TEXT](http://img.youtube.com/vi/9R55uCCrNss/0.jpg)](http://www.youtube.com/watch?v=9R55uCCrNss "GNSS-FGO Video Demonstration")

This is the official implementation of gnssFGO using the general framework onlineFGO based on GTSAM, in which a general time-centric factor graph optimization with continuous-time trajectory representation using Gaussian process regression for online applications is implemented. 
The goal of this framework is to build a fundamental time-centric graph-optimization state estimator for online applications while doing research on:
1. multi-sensor fusion to improve the robustness of vehicle localization in harsh environments
 1. Fusing both tightly coupled and loosely coupled GNSS observations
 2. Fusing lidar odometries
 3. Fusing visual odometries
 4. Fusing uwb
 5. etc.
2. advanced inference for e.g., online sensor noise identification and hyper-parameter tuning (onging works)

### Currently, only ROS2 is supported, the ROS1 version is in development.
### Call for collaborations, contact: h.zhang@irt.rwth-aaachen.de
---

## Dependencies (mostly as submodules):
1. [ros2](https://docs.ros.org/en/rolling/Installation.html)
2. [irt_nav_common](https://github.com/rwth-irt/irt_nav_common)
3. [irt_gnss_preprocessing](https://github.com/rwth-irt/irt_gnss_preprocessing)
5. [GTSAM](https://github.com/borglab/gtsam) 
6. [adapted LIO-SAM](https://github.com/rwth-irt/LIO-SAM)
4. [Eigen3](https://eigen.tuxfamily.org/)
5. [novatel_oem7_msgs](https://github.com/rwth-irt/novatel_oem7_driver)
6. [ublox_msgs](https://github.com/rwth-irt/ublox)
7. [ublox_serialization](https://github.com/rwth-irt/ublox)
8. [mapviz (optional)](https://github.com/rwth-irt/mapviz.git)
---

## Papers using this package
[1] *Haoming Zhang, Chih-Chun Chen, Heike Vallery and Timothy D. Barfoot*, GNSS/Multi-Sensor Fusion Using Continuous-Time Factor Graph Optimization for Robust Localization, submitted to IEEE T-TRO, arxiv, DOI: [10.48550/arXiv.2309.11134](https://doi.org/10.48550/arXiv.2309.11134)
### Data available at: https://rwth-aachen.sciebo.de/s/OCEZPLE9wFHv1pp

[2] *Haoming Zhang, Zhanxin Wang and Heike Vallery*, Learning-based NLOS Detection and Uncertainty Prediction of GNSS Observations with Transformer-Enhanced LSTM Network, accepted at the IEEE ITSC2023, arxiv, DOI: [10.48550/arXiv.2309.00480](https://arxiv.org/abs/2309.00480)

For more information and sample request, please contact h.zhang@irt.rwth-aaachen.de


---
## How to start
1. Download the [datasets](https://rwth-aachen.sciebo.de/s/OCEZPLE9wFHv1pp)
2. Install ros2 (recommend: rolling or humble)
3. Clone this repo with submodules in your colcon workspace 
```
git clone --recursive https://github.com/rwth-irt/gnssFGO.git
```
4. compile

## How to run
1. When using the datasets, launch irt_gnss_preprocess

  - For data in AC:
  ```
  ros2 launch irt_gnss_preprocessing gnss_preprocessor.launch.py
  ```
  - For the rest:
  ```
  ros2 launch irt_gnss_preprocessing deloco_preprecessor.launch.py 
  ```
2. Launch onlineFGO
  - For loosely coupled GNSS fusion AC:
  ```
  ros2 launch online_fgo aachen_lc.launch.py
  ```
  - For tightly coupled GNSS fusion AC:
  ```
  ros2 launch online_fgo aachen_tc.launch.py
  ```
3. Play ros bag
  ```
  ros2 bag play AC --clock --start-offset 60
  ```
4. (optional) Visualization in Mapviz (adapted from the official version)
  ```
  ros2 launch mapviz mapviz.launch.py
  ```
5. (optinal) If LIOIntegrator is used
  ```
  ros2 launch lio_sam fgorun.launch.py 
  ```

---
## ToDos:
1. Clean code base and move general utilities to irt_nav_common
2. Fix bugs for GNSS carrier phase integration
3. Implement visual odometry