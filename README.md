# Camera Lite
A simple ROS2 publisher, using OpenCV to capture a frame from the camera and publish.  This is a bare bones implimentation.  It does not do any camera calibration or other advanced stuff.  It just works.

Has been tested on a Raspberry Pi 3 with Ubuntu 20.04 LTS and ROS Foxy.

## Dependencies
In order to take a picture, a simple service definition is needed.  Please install [Camera Lite Interfaces](https://github.com/slaghuis/camera_lite_interfaces.git)

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

To run both the camera node, and the service to take a picture use the provided launch file.  Note the parameters in the launch file states the resolutions 640x380.  You can change this to a resolution supported by your camera and application.
```
ros2 launch camera.launch.py
```
Take a picture using this service call
```
ros2 service call /camera/save_picture camera_lite_interfaces/srv/Save "{name: /home/ubuntu/test.jpg}"
```
