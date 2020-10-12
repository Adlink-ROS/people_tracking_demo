# People Tracking Demo

## Prerequisite
- Ubuntu 18.04 Desktop
- ROS 2 Dashing
- OpenCV 3.3
- Intel® OpenVINO™ Toolkit 2019 R3.1 (2019.3.376)
- Intel® RealSense™ D435 and its library
---

## Install OpenVINO Toolkit

Download OpenVINO Toolkit from Intel [website](https://software.intel.com/en-us/openvino-toolkit) and install it. Please make sure you are using the same version of OpenVINO toolkit.

```bash
# extract OpenVINO toolkit 2019 R3.1 which downloaded from intel website
tar -xvzf l_openvino_toolkit_p_2019.3.376.tgz
cd l_openvino_toolkit_p_2019.3.376
sudo ./install_GUI.sh
```

## Install Dependencies of OpenVINO

After finished the installation, execute following commands

```bash
cd /opt/intel/openvino/install_dependencies
sudo -E ./install_openvino_dependencies.sh
```

## Install Intel® RealSense™ SDK 2.0

```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt install -y librealsense2-dkms \
librealsense2-utils \
librealsense2-dev \
librealsense2-dbg
```

 Reconnect your RealSense camera and run: `realsense-viewer` to verify the installation

![https://s3-us-west-2.amazonaws.com/secure.notion-static.com/fb0113ca-0f92-42fe-bf60-2af289925572/RS_D435_color_depth_LG.jpg](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/fb0113ca-0f92-42fe-bf60-2af289925572/RS_D435_color_depth_LG.jpg)

## Configure the Model Optimizer of OpenVINO

```bash
source /opt/intel/openvino/bin/setupvars.sh
cd /opt/intel/openvino/deployment_tools/model_optimizer/install_prerequisites
sudo ./install_prerequisites.sh
```

## Build and Install the Inference Engines of OpenVINO

**root** is required instead of sudo

```bash
sudo su
source /opt/intel/openvino/bin/setupvars.sh
cd /opt/intel/openvino/deployment_tools/inference_engine/samples/
mkdir build
cd build
cmake ..
make
exit
```

## Download and Build OpenCV 3.3

Prepare the dependencies before building OpenCV

```bash
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
 cd ~/code
```

Build OpenCV

```bash
 mkdir ~/opencv3
 cd ~/opencv3
 git clone https://github.com/opencv/opencv.git
 git clone https://github.com/opencv/opencv_contrib.git
 cd opencv && git checkout 3.4.2 && cd ..
 cd opencv_contrib && git checkout 3.4.2 && cd ..
 cd opencv
 mkdir build && cd build
 cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=$HOME/opencv3/opencv_contrib/modules/ ..
 make -j8
 sudo make install
```

## Setup OpenVINO Environment

Add below environment variables into ~/.openvino_bashrc

```bash
echo 'source /opt/intel/openvino/bin/setupvars.sh' > ~/.openvino_bashrc
echo "export CPU_EXTENSION_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so" >> ~/.openvino_bashrc
echo "export GFLAGS_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a"  >> ~/.openvino_bashrc
echo "export OpenCV_DIR=$HOME/opencv3/opencv/cmake" >> ~/.openvino_bashrc
```

## Install the Packages needed by People Tracking

### Download packages

```bash
mkdir -p ~/ros2_people_tracking_ws/src
cd ~/ros2_people_tracking_ws
wget https://raw.githubusercontent.com/Adlink-ROS/people_tracking_demo/dashing-devel/people_tracking_demo.repos
vcs import src < people_tracking_demo.repos
```

### Build packages

```bash
source /opt/ros/dashing/setup.bash
source ~/.openvino_bashrc
cd ~/ros2_people_tracking_ws
colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'
```

### Run People Tracking Packages

```bash
# Source all the environment
source /opt/ros/dashing/setup.bash
source ~/.openvino_bashrc
source ~/ros2_people_tracking_ws/install/local_setup.bash

# Bringup the packages
ros2 launch dynamic_vino_sample ros2_openvino_oa.launch.py
ros2 launch object_analytics_node object_analytics_sample.launch.py open_rviz:=true
ros2 launch people_tracking_launcher bringup_launch.py
```

### Bringup Your Robot

Finally, bringup your robot with ROS 2 drivers to receive cmd_vel. The robot will track people automatically!

Take OmniBot for example:

```bash
source /opt/ros/dashing/setup.bash
source ~/omnibot_ros2_ws/install/local_setup.bash
ros2 launch omni_base_driver bringup_launch.py
```
