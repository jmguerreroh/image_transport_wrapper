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

#include "image_transport_wrapper/WrapperPublisher.hpp"


namespace wrapper_publisher
{

WrapperPublisher::WrapperPublisher()
: Node("wrapper_publisher"), logger_(rclcpp::get_logger("WrapperPublisher"))
{
  // Declare the parameters
  this->declare_parameter("topic_in", "image");
  this->declare_parameter("topic_out", "wrapper/image");
  this->get_parameter("topic_in", topic_in_);
  this->get_parameter("topic_out", topic_out_);

  // Add a slash to the topic name if it does not have one
  if (topic_in_.find("/") != 0) {
    topic_in_.insert(0, "/");
  }

  // Get the list of topics
  auto topic_manager = this->get_topic_names_and_types();
  bool foundImageTopic = false;
  for (const auto & topic : topic_manager) {
    if (topic.first == topic_in_) {
      foundImageTopic = true;
      break;
    }
  }

  // Check if the topic does not exist and throw an exception
  if (!foundImageTopic) {
    RCLCPP_ERROR(logger_, "Topic '%s' does not exist.", topic_in_.c_str());
    throw rclcpp::exceptions::InvalidTopicNameError(topic_in_.c_str(), "Topic does not exist.", 1);
  }

  // Assign the subscriber to a raw transport
  const image_transport::TransportHints hints(this, "raw");
  try {
    auto subscription_options = rclcpp::SubscriptionOptions();
    // Create a subscription with QoS profile that will be used for the subscription.
    subscriber_ = image_transport::create_subscription(
      this,
      topic_in_,
      std::bind(&WrapperPublisher::imageCallback, this, std::placeholders::_1),
      hints.getTransport(),
      rmw_qos_profile_sensor_data,
      subscription_options);
  } catch (image_transport::TransportLoadException & e) {
    RCLCPP_ERROR(
      logger_, "Failed to create subscriber for topic %s: %s", topic_in_.c_str(),
      e.what());
  }

  // Create an ImageTransport instance, initializing it with a subnode
  rclcpp::Node::SharedPtr subnode = rclcpp::Node::create_sub_node("subnode");
  image_transport::ImageTransport it(subnode);
  // Create a publisher using ImageTransport to publish on the topic
  publisher_ = it.advertise(topic_out_, 1);

}


void WrapperPublisher::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Publish the image using the publisher created with ImageTransport
  if (publisher_.getNumSubscribers() > 0) {
    publisher_.publish(msg);
  }
}

}  // namespace wrapper_publisher
