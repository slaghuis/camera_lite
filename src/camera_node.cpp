// Copyright 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>     // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory>     // Dynamic memory management
#include <string>     // String functions

#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// #include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

// Global constants
static const int FRAME_WIDTH = 1024;
static const int FRAME_HEIGHT = 768;

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
  public:
    ImagePublisher()
    : Node("camera_node")
    {
      // Declare and get Parameters
      frame_width_ = this->declare_parameter<int>("frame_width", FRAME_WIDTH);
      frame_height_ = this->declare_parameter<int>("frame_height", FRAME_HEIGHT);
      device_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
      freq_ = this->declare_parameter("frequency", 30.0);
      frame_id_ = this->declare_parameter("frame_id", "camera");

      // Publisher publishes String messages to a topic named "addison".
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw",10);
    
      // Open the camera stream
      cap.open(device_id_, cv::CAP_V4L2);
      
      // Set the width and height based on command line arguments.
      cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(frame_width_));
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(frame_height_));
      
      if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        throw std::runtime_error("Could not open video stream");
      }

      // Initialize the timer. 
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)), 
        std::bind(&ImagePublisher::timer_callback, this));
    }

  private:
    // This method executes every 500 milliseconds
    void timer_callback()
    {

      cv::Mat frame;

      // Capture the next frame from the camera
      cap >> frame;

      cv_bridge::CvImage img_bridge;
      sensor_msgs::msg::Image img_msg; // >> message to be sent

      std_msgs::msg::Header header; // empty header
      header.stamp = this->get_clock()->now();
      header.frame_id = frame_id_;
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::msg::Image
      publisher_->publish(img_msg);
    }
 
    cv::VideoCapture cap;
    
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;

    // Declaration of the publisher_ attribute
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

   // Declaration of parameters
   int device_id_;
   size_t frame_width_;
   size_t frame_height_;
   double freq_;
   std::string frame_id_;
};

// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<ImagePublisher>());

  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
