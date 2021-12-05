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


#include <inttypes.h>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

// #include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>

#include <rclcpp/rclcpp.hpp>

#include "camera_lite_interfaces/srv/save.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using SavePicture = camera_lite_interfaces::srv::Save;

class PictureNode : public rclcpp::Node
{
public:
  PictureNode()
  : Node("picture_node"), save_next_image_(false)
  {    
    file_name_ = "undefined.jpg";
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&PictureNode::image_callback, this, std::placeholders::_1));
        
    service_ = create_service<SavePicture>("camera/save_picture", std::bind(&PictureNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));   
  }

private:
  
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(),
            "Image received\t Timestamp: %u.%u sec ",msg->header.stamp.sec,msg->header.stamp.nanosec);
    
    if( save_next_image_ ) {
      // Frame acquisition
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s",e.what());
        return;
      }

      cv::Mat image;
      cv_ptr->image.copyTo(image);   // BGR image coming from Raspberry Pi Camera via ROSinputVideo.retrieve(image);
    
      cv::imwrite(file_name_, image);  // Save the image to a JPG file.
      save_next_image_ = false;
    }
  }
  
  void handle_service(
  const std::shared_ptr<SavePicture::Request> request,
  const std::shared_ptr<SavePicture::Response> response)
  {
    file_name_ = request->name;
    save_next_image_ = true;
    response->result = true;    
  }

  // Private Variables ///////////////////////////////////////////////////////////////////////////  

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  
  rclcpp::Service<SavePicture>::SharedPtr service_;
  
  bool save_next_image_;
  std::string file_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PictureNode>());
  rclcpp::shutdown();
  return 0;
}
