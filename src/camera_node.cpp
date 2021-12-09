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
#include <map>

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

static
std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
  {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
  {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

static
std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
  {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
  {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
};

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
  public:
    ImagePublisher()
    : Node("camera_node")
    {
    
      // Parse 'reliability' parameter
      rcl_interfaces::msg::ParameterDescriptor reliability_desc;
      reliability_desc.description = "Reliability QoS setting for the image publisher";
      reliability_desc.additional_constraints = "Must be one of: ";
      for (auto entry : name_to_reliability_policy_map) {
        reliability_desc.additional_constraints += entry.first + " ";
      }
      const std::string reliability_param = this->declare_parameter(
        "reliability", "reliable", reliability_desc);
      auto reliability = name_to_reliability_policy_map.find(reliability_param);
      if (reliability == name_to_reliability_policy_map.end()) {
        std::ostringstream oss;
        oss << "Invalid QoS reliability setting '" << reliability_param << "'";
        throw std::runtime_error(oss.str());
      }
      reliability_policy_ = reliability->second;

      // Parse 'history' parameter
      rcl_interfaces::msg::ParameterDescriptor history_desc;
      history_desc.description = "History QoS setting for the image publisher";
      history_desc.additional_constraints = "Must be one of: ";
      for (auto entry : name_to_history_policy_map) {
        history_desc.additional_constraints += entry.first + " ";
      }
      const std::string history_param = this->declare_parameter(
        "history", name_to_history_policy_map.begin()->first, history_desc);
      auto history = name_to_history_policy_map.find(history_param);
      if (history == name_to_history_policy_map.end()) {
        std::ostringstream oss;
        oss << "Invalid QoS history setting '" << history_param << "'";
        throw std::runtime_error(oss.str());
      }
      history_policy_ = history->second;
    
      // Declare and get Parameters
      frame_width_ = this->declare_parameter<int>("frame_width", FRAME_WIDTH);
      frame_height_ = this->declare_parameter<int>("frame_height", FRAME_HEIGHT);
      device_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
      freq_ = this->declare_parameter("frequency", 30.0);
      frame_id_ = this->declare_parameter("frame_id", "camera");

      auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        history_policy_,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        depth_
      ));
      // The reliability policy can be reliable, meaning that the underlying transport layer will try
      // ensure that every message gets received in order, or best effort, meaning that the transport
      // makes no guarantees about the order or reliability of delivery.
      qos.reliability(reliability_policy_);
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);
    
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
      
      // If no frame was grabbed, return early
      if (frame.empty()) {
        return;
      }

      cv_bridge::CvImage img_bridge;
      sensor_msgs::msg::Image img_msg; // >> message to be sent

      std_msgs::msg::Header header; // empty header
      header.stamp = this->get_clock()->now();
      header.frame_id = frame_id_;
      
      //sensor_msgs::msg::Image container(frame, header);
      //publisher_->publish(std::move(container));
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
   size_t depth_;
   double freq_;
   rmw_qos_reliability_policy_t reliability_policy_;
   rmw_qos_history_policy_t history_policy_;
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
