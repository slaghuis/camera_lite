# Camera Lite
A simple ROS2 publisher, using OpenCV to capture a frame from the camera and publish.  This is a bare bones implimentation.  It does not do any camera calibration or other advanced stuff.  It just works.

Has been tested on a Raspberry Pi 3 with Ubuntu 20.04 LTS and ROS Foxy.

## Installation
MAKE SURE YOUR CAMERA HAS BEEN ENABLED.  (Out of scope for write-up)

Install OpenCV
```
sudo apt update
sudo apt install libopencv-dev python3-opencv
```
Install the ROS - OpenCV Bridge and the vision library
```
sudo apt-get install ros-foxy-cv-bridge
sudo apt-get install ros-foxy-vision-opencv
```
Clone this repo into the src directory of your ROS2 workspace. See the [ros2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) on how to create a workspace.
```
git clone https://github.com/slaghuis/camera_lite.git
```
Back in the root of your ROS workspace, build and install the package.  
```
colcon build --packages-select camera_lite
. install/setup.bash
```
Run the camera node.  To test with image_tools the topic has to be remapped to /image
```
ros2 run camera_lite camera_node --ros-args --remap /camera/image_raw:=/image
```
Run image_tools showimage to see the camera output.
```
ros2 run image_tools showimage
```
