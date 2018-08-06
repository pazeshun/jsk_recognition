// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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

#include <algorithm>
#include <jsk_pcl_ros_utils/extract_sub_polygon_array.h>

namespace jsk_pcl_ros_utils
{
  void ExtractSubPolygonArray::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("indices", indices_, std::vector<int>());
    pub_polygons_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output_polygons", 1);
    pub_coefficients_
      = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
        *pnh_,
        "output_coefficients", 1);
  }

  void ExtractSubPolygonArray::subscribe()
  {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_->connectInput(sub_polygons_, sub_coefficients_);
    sync_->registerCallback(boost::bind(
                              &ExtractSubPolygonArray::extract,
                              this, _1, _2));
  }

  void ExtractSubPolygonArray::unsubscribe()
  {
    sub_polygons_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }

  void ExtractSubPolygonArray::extract(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& sub_polygons_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& sub_coefficients_msg)
  {
    if (sub_polygons_msg->polygons.size() != sub_coefficients_msg->coefficients.size())
    {
      NODELET_ERROR_THROTTLE(10.0, "Polygons size and coefficients size are different");
      return;
    }
    jsk_recognition_msgs::PolygonArray pub_polygons_msg;
    jsk_recognition_msgs::ModelCoefficientsArray pub_coefficients_msg;
    pub_polygons_msg.header = sub_polygons_msg->header;
    pub_coefficients_msg.header = sub_coefficients_msg->header;
    for (int i = 0; i < indices_.size(); i++)
    {
      int idx = indices_[i];
      if (idx < 0)
      {
        // To support minus index
        idx += sub_polygons_msg->polygons.size();
      }
      if (0 <= idx && idx < sub_polygons_msg->polygons.size())
      {
        pub_polygons_msg.polygons.push_back(sub_polygons_msg->polygons[idx]);
        if (0 <= idx && idx < sub_polygons_msg->labels.size())
        {
          pub_polygons_msg.labels.push_back(sub_polygons_msg->labels[idx]);
          // Skip if labels array is not provided
        }
        if (0 <= idx && idx < sub_polygons_msg->likelihood.size())
        {
          pub_polygons_msg.likelihood.push_back(sub_polygons_msg->likelihood[idx]);
          // Skip if likelihood array is not provided
        }
        pub_coefficients_msg.coefficients.push_back(sub_coefficients_msg->coefficients[idx]);
      }
      else
      {
        NODELET_ERROR_THROTTLE(10.0, "Invalid index %d is specified for polygons whose size is %d", indices_[i],
                               (int)sub_polygons_msg->polygons.size());
        return;
      }
    }
    pub_polygons_.publish(pub_polygons_msg);
    pub_coefficients_.publish(pub_coefficients_msg);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::ExtractSubPolygonArray, nodelet::Nodelet);
