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
      // Declare Parameters
      this->declare_parameter<int>("frame_width", FRAME_WIDTH);
      this->declare_parameter<int>("frame_height", FRAME_HEIGHT);

      // Read from parameters
      this->get_parameter("frame_width", frame_width_);
      this->get_parameter("frame_height", frame_height_);

      // Publisher publishes String messages to a topic named "addison".
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw",10);

      // Initialize the timer. The timer_callback function will execute every
      // 500 milliseconds.
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ImagePublisher::timer_callback, this));
    }

  private:
    // This method executes every 500 milliseconds
    void timer_callback()
    {

      // Open the camera
      cv::VideoCapture camera(0,cv::CAP_V4L2);

      if (!camera.isOpened()) {
        RCLCPP_ERROR(this->get_logger(),"ERROR: Could not open camera");
        return;
      }
      // camera.set( cv::CAP_PROP_FORMAT, cv::CAP_MODE_BGR);
      camera.set( cv::CAP_PROP_FRAME_WIDTH, frame_width_);
      camera.set( cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
      cv::Mat frame;

      // Capture the next frame from the camera
      camera >> frame;

      cv_bridge::CvImage img_bridge;
      sensor_msgs::msg::Image img_msg; // >> message to be sent

      std_msgs::msg::Header header; // empty header
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera";
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::msg::Image
      publisher_->publish(img_msg);
    }

    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;

    // Declaration of the publisher_ attribute
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

   // Declaration of parameters
   int frame_width_;
   int frame_height_;
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
