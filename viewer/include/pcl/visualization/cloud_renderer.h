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

#ifndef PCL_VISUALIZATION_CLOUD_RENDERER_H_
#define PCL_VISUALIZATION_CLOUD_RENDERER_H_

#include <QVTKWidget.h>
#include <QObject>

#include <pcl/io/grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include<pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <boost/thread.hpp>

namespace pcl {
namespace visualization {
/** \brief Abstract base class for rendering a cloud in a CloudPlayerWidget
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup visualization
 */
class CloudRenderer : public QObject {
Q_OBJECT
 protected:
  std::string description_;
  QVTKWidget* widget_;
  pcl::visualization::PCLVisualizer* visualizer_;
  boost::signals2::connection connection_;
 public:
  CloudRenderer()
      : widget_(NULL),
        visualizer_(NULL) {
  }
  virtual ~CloudRenderer() {
  }

  /* \brief initialize with visualizer and widget */
  virtual void init(QVTKWidget* widget, PCLVisualizer * visualizer) {
    widget_ = widget;
    visualizer_ = visualizer;
  }

  /*\brief setup renderer for rendering grabber point clouds */
  virtual bool
  setup(boost::shared_ptr<Grabber>& grabber)=0;

  /* \brief stops rendering and disconnects from grabber */
  virtual void
  disconnect()=0;

  /* \brief manually set a new cloud rather than receiving it from the grabber */
  virtual void
  setCloud(const pcl::PCLPointCloud2::ConstPtr& cloud)=0;

  /* \brief tells the PCLVisualizer to render  a new point cloud if one is present */
  virtual void
  renderNew()=0;

  /* \brief returns description of renderer */
  std::string description() {
    return description_;
  }

signals:
  void update();
};

/** \brief Renders a point field along a color map range in a CloudPlayerWidget
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup visualization
 */
class CloudRendererRange : public CloudRenderer {
 public:
  CloudRendererRange(std::string field);

  virtual ~CloudRendererRange() {
  }

  virtual bool
  setup(boost::shared_ptr<Grabber>& grabber);

  virtual void
  disconnect();

  virtual void
  renderNew();

  void setCloud(const pcl::PCLPointCloud2::ConstPtr& cloud) {
    cloud_ = cloud;
  }

 private:
  std::string field_name_;
  boost::mutex cloud_mutex_;

  pcl::PCLPointCloud2::ConstPtr cloud_;

  bool valid_grabber_;
  void grabberCB(const pcl::PCLPointCloud2::ConstPtr& cloud);
  boost::signals2::connection connection_;
};

/** \brief Renders intensity in black and white in a CloudPlayerWidget
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup visualization
 */
class CloudRendererBW : public CloudRenderer {
 public:
  CloudRendererBW();
  virtual ~CloudRendererBW() {
  }
  virtual bool
  setup(boost::shared_ptr<Grabber>& grabber);

  virtual void
  disconnect();

  virtual void
  renderNew();

  void
  setCloud(const pcl::PCLPointCloud2::ConstPtr& cloud);

 private:
  boost::mutex cloud_mutex_;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_;
  bool valid_grabber_;
  void grabberCB(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  boost::signals2::connection connection_;
};

class CloudRendererIZ : public CloudRenderer {
 public:
  CloudRendererIZ();
  virtual ~CloudRendererIZ() {
  }
  virtual bool
  setup(boost::shared_ptr<Grabber>& grabber);

  virtual void
  disconnect();

  virtual void
  renderNew();

  void
  setCloud(const pcl::PCLPointCloud2::ConstPtr& cloud);

 private:
  boost::mutex cloud_mutex_;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_;
  bool valid_grabber_;
  void grabberCB(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  boost::signals2::connection connection_;
};
}
}

#endif /* PCL_VISUALIZATION_CLOUD_RENDERER_H_ */
