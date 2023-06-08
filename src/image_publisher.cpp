/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, José Miguel Guerrero Hernández.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <fstream>

int main(int argc, char ** argv)
{
  // Create a logger
  rclcpp::Logger logger_ = rclcpp::get_logger("image_publisher");

  // Check if the image file exists
  std::string filePath = argv[1];
  std::ifstream file(filePath);
  if (!file) {
    RCLCPP_ERROR(logger_, "File not found: '%s'", filePath.c_str());
    return -1;
  }

  // Initialize ROS and create a node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("image_publisher", options);

  // Declare a parameter of type string with a default value
  node_->declare_parameter<std::string>("topic_image", "image");

  // Get the value of the parameters
  std::string topic_image_;
  node_->get_parameter("topic_image", topic_image_);

  // Print the value of the parameters
  RCLCPP_INFO(logger_, "Publishing '%s' on '%s' topic.", filePath.c_str(), topic_image_.c_str());

  // Create an ImageTransport instance, initializing it with our Node
  image_transport::ImageTransport it(node_);
  // Create a publisher using ImageTransport to publish on the topic
  image_transport::Publisher pub = it.advertise(topic_image_, 1);

  // Publish the image
  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok()) {
    if (pub.getNumSubscribers() > 0) {
      // OpenCV Mat image
      cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);
      // Convert the image to a ROS message
      std_msgs::msg::Header hdr;
      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
        hdr,
        sensor_msgs::image_encodings::BGR8,
        image).toImageMsg();
      pub.publish(msg);
    }
    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
}
