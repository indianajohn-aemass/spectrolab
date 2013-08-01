/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#include <pcl/visualization/cloud_renderer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/visualization/bw_color_handler.h>

pcl::visualization::CloudRendererRange::CloudRendererRange(std::string field)
    : valid_grabber_(false),
      field_name_(field) {
  description_ = "Color by ";
  description_ = description_ + field + " field";
}

bool pcl::visualization::CloudRendererRange::setup(
    boost::shared_ptr<Grabber>& grabber) {

  valid_grabber_ = false;
  typedef void (sig_cb)(const sensor_msgs::PointCloud2::ConstPtr&);

  if (!grabber->providesCallback<sig_cb>())
    return false;

  valid_grabber_ = true;
  connection_ = this->connection_ = grabber->registerCallback<sig_cb>(
      boost::bind(&CloudRendererRange::grabberCB, this, _1));
  return true;
}

void pcl::visualization::CloudRendererRange::disconnect() {
  connection_.disconnect();
  valid_grabber_ = false;
  cloud_.reset();
}

void pcl::visualization::CloudRendererRange::renderNew() {
  if (!valid_grabber_)
    return;
  boost::unique_lock<boost::mutex> lock(cloud_mutex_);
  if (cloud_ == NULL)
    return;
  pcl::visualization::PointCloudColorHandlerGenericField<
      sensor_msgs::PointCloud2>::Ptr chand(
      new pcl::visualization::PointCloudColorHandlerGenericField<
          sensor_msgs::PointCloud2>(cloud_, field_name_));

  Eigen::Vector4f origin(0, 0, 0, 1);
  Eigen::Quaternionf rot = Eigen::Quaternionf::Identity();
  this->visualizer_->removeAllPointClouds();
  this->visualizer_->addPointCloud(cloud_, chand, origin, rot);
  widget_->update();
  cloud_.reset();
}

void pcl::visualization::CloudRendererRange::grabberCB(
    const sensor_msgs::PointCloud2::ConstPtr& cloud) {
  {
    boost::unique_lock<boost::mutex> lock(cloud_mutex_);
    cloud_ = cloud;
  }
  emit update();
}

/************************************************************************/

pcl::visualization::CloudRendererBW::CloudRendererBW()
    : valid_grabber_(false) {
  description_ = "Black/White intensity coloring";
}

bool pcl::visualization::CloudRendererBW::setup(
    boost::shared_ptr<Grabber>& grabber) {

  valid_grabber_ = false;
  typedef void (sig_cb)(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&);

  valid_grabber_ = grabber->providesCallback<sig_cb>();
  if (!valid_grabber_)
    return false;

  connection_ = this->connection_ = grabber->registerCallback<sig_cb>(
      boost::bind(&CloudRendererBW::grabberCB, this, _1));
  return true;
}

void pcl::visualization::CloudRendererBW::disconnect() {
  connection_.disconnect();
  valid_grabber_ = false;
  cloud_.reset();
}

void pcl::visualization::CloudRendererBW::renderNew() {
  if (!valid_grabber_)
    return;
  boost::unique_lock<boost::mutex> lock(cloud_mutex_);
  if (cloud_ == NULL)
    return;
  pcl::visualization::PointCloudIntensityHandler<PointXYZI>::Ptr chand(
      new pcl::visualization::PointCloudIntensityHandler<PointXYZI>(cloud_));

  this->visualizer_->removeAllPointClouds();
  this->visualizer_->addPointCloud<pcl::PointXYZI>(cloud_, *chand);
  this->visualizer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4);
  widget_->update();
  cloud_.reset();
}

void pcl::visualization::CloudRendererBW::grabberCB(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
  {
    boost::unique_lock<boost::mutex> lock(cloud_mutex_);
    cloud_ = cloud;
  }
  emit update();
}

void pcl::visualization::CloudRendererBW::setCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr tcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud, *tcloud);
  cloud_ = tcloud;
}

/*************************************************************************/

#include "pcl/visualization/intensity_range_handler.h"

pcl::visualization::CloudRendererIZ::CloudRendererIZ()
    : valid_grabber_(false) {
  description_ = "Range/intensity coloring";
}

bool pcl::visualization::CloudRendererIZ::setup(
    boost::shared_ptr<Grabber>& grabber) {

  valid_grabber_ = false;
  typedef void (sig_cb)(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&);

  valid_grabber_ = grabber->providesCallback<sig_cb>();
  if (!valid_grabber_)
    return false;

  connection_ = this->connection_ = grabber->registerCallback<sig_cb>(
      boost::bind(&CloudRendererIZ::grabberCB, this, _1));
  return true;
}

void pcl::visualization::CloudRendererIZ::disconnect() {
  connection_.disconnect();
  valid_grabber_ = false;
  cloud_.reset();
}

void pcl::visualization::CloudRendererIZ::renderNew() {
  if (!valid_grabber_)
    return;
  boost::unique_lock<boost::mutex> lock(cloud_mutex_);
  if (cloud_ == NULL)
    return;
  pcl::visualization::PointCloudIZHandler<PointXYZI>::Ptr chand(
      new pcl::visualization::PointCloudIZHandler<PointXYZI>(cloud_));

  this->visualizer_->removeAllPointClouds();
  this->visualizer_->addPointCloud<pcl::PointXYZI>(cloud_, *chand);
  this->visualizer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4);
  widget_->update();
  cloud_.reset();
}

void pcl::visualization::CloudRendererIZ::grabberCB(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
  {
    boost::unique_lock<boost::mutex> lock(cloud_mutex_);
    cloud_ = cloud;
  }
  emit update();
}

void pcl::visualization::CloudRendererIZ::setCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr tcloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud, *tcloud);
  cloud_ = tcloud;
}
