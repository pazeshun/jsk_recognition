// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#include "jsk_pcl_ros/pointcloud_to_cluster_point_indices.h"

namespace jsk_pcl_ros
{
  void PointCloudToClusterPointIndices::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
  }

  void PointCloudToClusterPointIndices::subscribe()
  {
    sub_ = pnh_->subscribe(
      "input", 1, &PointCloudToClusterPointIndices::convert, this);
  }

  void PointCloudToClusterPointIndices::unsubscribe()
  {
    sub_.shutdown();
  }

  void PointCloudToClusterPointIndices::convert(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    int point_num = msg->width * msg->height;
    pcl_msgs::PointIndices indices;
    jsk_recognition_msgs::ClusterPointIndices cluster_indices;
    for (int i = 0; i < point_num; i++) {
      indices.indices.push_back(i);
    }
    indices.header = msg->header;
    cluster_indices.header = msg->header;
    cluster_indices.cluster_indices.push_back(indices);
    pub_.publish(cluster_indices);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PointCloudToClusterPointIndices,
                        nodelet::Nodelet);
