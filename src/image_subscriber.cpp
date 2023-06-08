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
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

std::string transport_;
auto logger_ = rclcpp::get_logger("image_subscriber");

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    // Show the image using OpenCV
    cv::imshow(transport_, cv_bridge::toCvShare(msg, msg->encoding.c_str())->image);
    cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(logger_, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create a node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("image_subscriber", options);

  // Declare a parameter of type string with a default value
  node_->declare_parameter<std::string>("transport", "raw");
  node_->declare_parameter<std::string>("topic_image", "image");

  // Get the value of the parameters
  node_->get_parameter("transport", transport_);
  std::string topic_image_;
  node_->get_parameter("topic_image", topic_image_);
  // Add a slash to the topic name if it does not have one
  if (topic_image_.find("/") != 0) {
    topic_image_.insert(0, "/");
  }

  // Get the list of topics
  auto topic_manager = node_->get_topic_names_and_types();
  bool foundImageTopic = false;
  for (const auto & topic : topic_manager) {
    if (topic.first == topic_image_) {
      foundImageTopic = true;
      break;
    }
  }

  // Check if the topic exists
  if (!foundImageTopic) {
    RCLCPP_ERROR(logger_, "Topic '%s' does not exist.", topic_image_.c_str());
    return -1;
  }

  // Create a window to show the image
  cv::namedWindow(transport_);
  cv::startWindowThread();

  // Create an ImageTransport instance, initializing it with the Node
  image_transport::Subscriber subscriber_;
  // Assign the subscriber to a specific transport
  const image_transport::TransportHints hints(node_.get(), transport_);
  try {
    auto subscription_options = rclcpp::SubscriptionOptions();
    // Create a subscription with QoS profile that will be used for the subscription.
    subscriber_ = image_transport::create_subscription(
      node_.get(),
      topic_image_,
      std::bind(&imageCallback, std::placeholders::_1),
      hints.getTransport(),
      rmw_qos_profile_sensor_data,
      subscription_options);
    RCLCPP_INFO(
      logger_, "Image received: unwrapping using '%s' transport.",
      subscriber_.getTransport().c_str());
  } catch (image_transport::TransportLoadException & e) {
    RCLCPP_ERROR(
      logger_, "Failed to create subscriber for topic %s: %s",
      topic_image_.c_str(), e.what());
    return -1;
  }

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node_);
  cv::destroyWindow(transport_);

  return 0;
}
