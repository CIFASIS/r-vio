# R-VIO

R-VIO is an efficient, lightweight, **robocentric visual-inertial odometry** algorithm for consistent 3D motion tracking using only a monocular camera and a 6-axis IMU. Different from standard world-centric VINS algorithms which directly estimate absolute motion of the sensing platform with respect to a fixed, gravity-aligned, global frame of reference, R-VIO estimates the relative motion of higher accuracy with respect to a moving, local frame (for example, IMU frame) and updates global pose (orientation and position) estimate through composition. This code is developed with the robocentric sliding-window filtering-based VIO framework that was originally proposed in our *IROS2018* paper and further extended in our recent *IJRR* paper:

- Zheng Huai and Guoquan Huang, **Robocentric visual-inertial odometry**, *The International Journal of Robotics Research (IJRR)*, July 2019: [here](https://journals.sagepub.com/doi/10.1177/0278364919853361).

- Zheng Huai and Guoquan Huang, **Robocentric visual-inertial odometry**, *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Madrid, Spain, Oct 1-5, 2018: [here](https://ieeexplore.ieee.org/document/8593643).

Video (on **EuRoC** and our handheld datasets): https://www.youtube.com/watch?v=UtiZ0EKa55M.
![](https://media.giphy.com/media/RMecOYlfxEcy4T8JdS/giphy.gif)

Video (on our 9.8km **urban driving** dataset): https://www.youtube.com/watch?v=l9IC2ddBEYQ.
![](rvio.gif)

## Original repository
This repository is a modified version of [R-VIO](https://github.com/rpng/R-VIO) (see original README below). We facilitate the installation process and the use of Docker.

## Docker support

In order to facilitate the installation process, the system is wrapped up using Docker.
We provide scripts to create a Docker image, build the system and run it in a Docker container. 

### Dependencies 
* Docker
* ROS
* [`pose_listener`](https://github.com/CIFASIS/pose_listener) (if you use `run_rosario_sequence.sh`, see below)

### Building the system
Run:
```
./run.sh -b
```

This command creates a Docker image, installs all the dependencies and builds the system. The resulting image contains a version of the system ready to be run.

### Running the system in VIS mode
If you are not interested in making changes in the source code, you should run the system in VIS mode. Run:
```
./run.sh -v
```
The system is launched in a Docker container based on the previously built image. By default, this command executes a launch file which is configured to run the Rosario dataset. If you want to run your own dataset, **write a launch file and placed it in the `launch/` folder**. Configuration files must be placed in the `config/` folder. Then, run the script with the option `-l <LAUNCH_FILE_NAME>`. For example, if you are testing EuRoC, write `euroc_dataset.launch`, move it into `launch/` and type:
```
./run.sh -v -l euroc_dataset.launch
```
Making changes in launch/configuration files in the host is possible because these folders are mounted into the Docker container. It is not necessary to access the container through a bash shell to modify these files.

See below for information about input data and visualization.

### Running the system in DEV mode
DEV mode allows developers to make changes in the source code, recompile the system and run it with the modifications. To do this, the whole repository is mounted in a container. Run:
```
./run.sh -d
```
This opens a bash shell in a docker container. You can edit source files in the host and after that you can use this shell to recompile the system. When the compilation process finishes, you can run the method using `roslaunch`.

See below for information about input data and visualization.

### Input data and visualization

At this point, the system is waiting for input data. Either you can run `rosbag play` or you can use `run_rosario_sequence.sh`.
If you choose the latter, open a second terminal and run:
```
./run_rosario_sequence.sh -o <OUTPUT_TRAJECTORY_FILE> <ROSBAG_FILE>
```
In contrast to what `run.sh` does, `run_rosario_sequence.sh` executes commands in the host (you can modify it to use a Docker container). 

`ROSBAG_FILE` is played using `rosbag`. Also, make sure you have cloned and built `pose_listener` in your catkin workspace. Default path for the workspace is `${HOME}/catkin_ws`, set `CATKIN_WS_DIR` if the workspace is somewhere else (e.g.: `export CATKIN_WS_DIR=$HOME/foo_catkin_ws`). `pose_listener` saves the estimated trajectory in `<OUTPUT_TRAJECTORY_FILE>` (use absolute path). You can edit `run_rosario_sequence.sh` if you prefer to save the trajectory using your own methods. Additionally, `run_rosario_sequence.sh` launches `rviz` to display visual information during the execution of the system.

Alternatively, if you are not interested in development but in testing or visualization, instead of running `run.sh` and `run_rosario_sequence.sh` in two different terminals, you can just run:
```
./run_rosario_sequence.sh -r -o <OUTPUT_TRAJECTORY_FILE> <ROSBAG_FILE>
```
This launches a Docker container and executes the default launch file (see `LAUNCH_FILE` in `run.sh`). After that, the bagfile is played and `rviz` and `pose_listener` are launched. Add `-b` if you want to turn off the visualization.


# Original README

## 1. Prerequisites

We have tested the code under Ubuntu **16.04** and ROS **Kinetic**.

### ROS
Download and install instructions can be found at: http://wiki.ros.org/kinetic/Installation/Ubuntu.

Additional ROS packages needed: tf, sensor_msgs, geometry_msgs, nav_msgs, cv_bridge, eigen_conversions.

### Eigen
Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### OpenCV
Download and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3**. **Tested with 2.4.11 and 3.3.1**.

## 2. Build and Run
First, `git clone` the repository and `catkin_make` it. Then, to run `rvio` with single camera/IMU inputs from the ROS topics `/camera/image_raw` and `/imu`, a config file in *config* folder and the corresponding launch file in *launch* folder (for example, `rvio_euroc.yaml` and `euroc.launch` are for [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset) are needed, and to visualize the outputs of R-VIO please use `rviz` with the settings file `rvio_rviz.rviz` in *config* folder.
  ```
  Terminal 1: roscore
  ```
  ```
  Terminal 2: rviz (AND OPEN rvio_rviz.rviz IN THE CONFIG FOLDER)
  ```
  ```
  Terminal 3: roslaunch rvio euroc.launch
  ```
  ```
  Terminal 4: rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/image_raw /imu0:=/imu
  ```
You can also run R-VIO with your own sensor (data) by creating a config file `rvio_NAME_OF_YOUR_DATA.yaml` in *config* folder and the corresponding launch file `NAME_OF_YOUR_DATA.launch` in *launch* folder referring to the above EuRoC example.

## 3. License

The source code is released under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html) license.

We are still working on improving the code reliability. For any technical issue, please contact Zheng Huai: <zhuai@udel.edu>.

For commercial inquiries, please contact Guoquan (Paul) Huang: <ghuang@udel.edu>.

