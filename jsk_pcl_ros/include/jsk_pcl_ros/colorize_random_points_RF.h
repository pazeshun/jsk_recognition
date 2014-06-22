// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, yuto_inagaki and JSK Lab
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

#ifndef __JSK_PCL_COLORIZE_RANDOM_FOREST__
#define __JSK_PCL_COLORIZE_RANDOM_FOREST__

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <jsk_pcl_ros/PointsArray.h>
// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/fpfh_omp.h>
#include <ml_classifiers/ClassifyData.h>
#include <ml_classifiers/ClassDataPoint.h>
#include <math.h>
#include <stdlib.h>

namespace jsk_pcl_ros
{
  class ColorizeMapRandomForest: public pcl_ros::PCLNodelet
  {
  protected:
    ros::Subscriber sub_input_;
    ros::Publisher pub_;

    double angular_threshold_;
    double mps_distance_threshold_;
    double approx_threshold_;
    double max_depth_change_factor_;
    double normal_smoothingsize_;
    double refinement_threshold_;
    float radius_search_;
    float pass_offset_;
    float pass_offset2_;
    int min_inliers_;
    int mode_;
    int sum_num_;
  private:
    virtual void onInit();
    void extract(const sensor_msgs::PointCloud2 cloud);
  };
}

#endif
