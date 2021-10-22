/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/
#include <string>
#include <functional>
#include "zbar_ros/barcode_reader_node.hpp"
#include "cv_bridge/cv_bridge.h"

BarcodeReaderNode::BarcodeReaderNode()
: Node("BarcodeReader")
{
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);


  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image", 10, std::bind(&BarcodeReaderNode::imageCb, this, std::placeholders::_1));

  // barcode_pub_ = this->create_publisher<std_msgs::msg::String>("barcode", 10);
  barcode_pub_ = this->create_publisher<zbar_ros_interfaces::msg::Symbol>("barcode", 10);
}

void BarcodeReaderNode::imageCb(sensor_msgs::msg::Image::ConstSharedPtr image)
{
  RCLCPP_DEBUG(get_logger(), "Image received on subscribed topic");

  cv_bridge::CvImageConstPtr cv_image;
  cv_image = cv_bridge::toCvShare(image, "mono8");

  zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,
    cv_image->image.cols * cv_image->image.rows);
  scanner_.scan(zbar_image);

  auto it_start = zbar_image.symbol_begin();
  auto it_end = zbar_image.symbol_end();
  if (it_start != it_end) {
    // If there are barcodes in the image, iterate over all barcode readings from image
    for (zbar::Image::SymbolIterator symbol_it = it_start; symbol_it != it_end; ++symbol_it) {
      zbar_ros_interfaces::msg::Symbol symbol;

      symbol.data = symbol_it->get_data();
      RCLCPP_DEBUG(get_logger(), "Barcode detected with data: '%s'", symbol.data.c_str());

      RCLCPP_DEBUG(
        get_logger(), "Polygon around barcode has %d points", symbol_it->get_location_size());
      for (zbar::Symbol::PointIterator point_it = symbol_it->point_begin();
        point_it != symbol_it->point_end();
        ++point_it)
      {
        geometry_msgs::msg::Point point;
        point.x = (*point_it).x;
        point.y = (*point_it).y;
        RCLCPP_DEBUG(get_logger(), "  Point: %f, %f", point.x, point.y);
        symbol.points.push_back(point);
      }

      // publish symbol
      RCLCPP_DEBUG(get_logger(), "Publishing Symbol");
      barcode_pub_->publish(symbol);
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "No barcode detected in image");
  }

  zbar_image.set_data(NULL, 0);
}
