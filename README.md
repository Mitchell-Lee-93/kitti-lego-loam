# LeGO-LOAM for kitti dataset

This repository contains the modified code of LeGO-LOAM to run and evaluate with the kitti-data set. When you run the code, you'll get the trajectory results of LeGO-LOAM in KITTI ground-truth format and you can directly evaluate the result with KITTI ground-truth by EVO-eval kit. Wish you find it helpful, especially for those who are not familiar with ROS and LOAM.

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo and kinetic)
- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)

 ```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  sudo make install
  ```

## Compile

1. You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/Mitchell-Lee-93/kitti-lego-loam.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Making new bagfile from kitti dataset 
Download odometry dataset(color or gray, velodyne, calibration, ground truth)
from : http://www.cvlibs.net/datasets/kitti/eval_odometry.php and Merge them all in one dataset directory

1. Edit the launch file
```
gedit ~/catkin_ws/src/kittibag/launch/kittibag.launch
```
Change 'dataset_folder' and 'output_bag_file' to your own directories

2. Run the launch file:
```
roslaunch kittibag kittibag.launch
```
## Run the package
1. Before run, you should change the directory of the result files
```
gedit ~/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/launch/run.launch
```
change 'RESULT_PATH' to your result dir

2. Run the launch file:
```
roslaunch lego_loam run.launch
```
Notes: The parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.

3. Play existing bag files:
```
rosbag play *.bag --clock 
```

## Evaluation with evo kit
Check and follow this repository

https://github.com/MichaelGrupp/evo

## Evaluation results

<img src = "https://raw.githubusercontent.com/Mitchell-Lee-93/kitti-lego-loam/master/kittibag/pic/1.png" width = "300"> <img src = "https://raw.githubusercontent.com/Mitchell-Lee-93/kitti-lego-loam/master/kittibag/pic/2.png" width = "300">  <img src = "https://raw.githubusercontent.com/Mitchell-Lee-93/kitti-lego-loam/master/kittibag/pic/3.png" width = "200">

## For A-LOAM with kitti 
check https://github.com/Mitchell-Lee-93/kitti-A-LOAM

## Original code from
https://github.com/RobustFieldAutonomyLab/LeGO-LOAM

Modified code

1. utility.h

for Velodyne 64 channel
```
extern const string pointCloudTopic = "/kitti/velo/pointcloud"; <- you should check your own bag file topic

//param for vel-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 50;
```
2. featureAssociation.cpp

Since kitti data already have removed the distortion
```
float s 10 * (pi->intensity - int(pi->intensity)); -> float s = 1;

// to delete all the code that corrects point cloud distortion
TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]); -> removed
TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]); -> removed

*Notes: The parameter "loopClosureEnableFlag" is set to "true" for SLAM. 
```
3. transformfusion.cpp

To correct two diffrent TF of lego-loam results and kitti gt. And also to save the results in kitti gt format
```
From line 222 to 286, saving results code added
```

Reference : https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/12

