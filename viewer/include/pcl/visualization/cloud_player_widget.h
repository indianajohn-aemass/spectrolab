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

#ifndef PCL_VISUALIZATION_CLOUD_PLAYER_WIDGET_H_
#define PCL_VISUALIZATION_CLOUD_PLAYER_WIDGET_H_

#include "ui_cloud_player.h"
#include <pcl/io/grabber.h>
#include <pcl/io/recorder.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "cloud_renderer.h"
#include <QWidget>

class QErrorMessage;

namespace pcl {
namespace visualization {
/** \brief Widget for playing/recording point cloud streams
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup visualization
 */

class CloudPlayerWidget : public QWidget {
Q_OBJECT

 public:
  static float br_val;
  CloudPlayerWidget(QWidget* parent = 0, Qt::WindowFlags f = 0);
  virtual ~CloudPlayerWidget();

  void
  setGrabber(boost::shared_ptr<Grabber>& grabber);

  void
  addCloudRenderer(CloudRenderer* renderer);

  size_t getNumRenderers() {
    return renderers_.size();
  }

  uint32_t currentRendererIDX() {
    return current_renderer_idx_;
  }

  CloudRenderer*
  getRenderer(size_t idx);

  void
  setCurrentRenderer(int idx);

  void
  addRecorder(Recorder* recorder);

  uint32_t currentRecorderIDX() {
    return current_recorder_idx_;
  }

  void
  setCurrentRecorder(uint32_t idx);

 private:
  boost::shared_ptr<Grabber> grabber_;
  bool is_movie_grabber_;
  PCLVisualizer* pcl_visualizer_;

  void keyboardCB(const pcl::visualization::KeyboardEvent& event);

  Ui_CloudPlayer ui_;
  std::vector<CloudRenderer*> renderers_;
  std::vector<Recorder*> recorders_;

  uint32_t current_renderer_idx_;
  uint32_t current_recorder_idx_;

  bool recording_;
  bool playing_;
  boost::signals2::connection progress_connection_;

  QErrorMessage* error_msg_;

  void cacheCloud(const sensor_msgs::PointCloud2ConstPtr& cloud);
  sensor_msgs::PointCloud2ConstPtr cached_cloud_;
  boost::signals2::connection cache_connection_;

 public slots:
  void playPause();
  void sliderValueChanged(int val);
  void record();
  void readBrightSlider(int val);
  void resetView();	// rm
  void updateCloud();
 protected:
  void enablePlayback();
  void disablePlayback();
  void progressUpdate(size_t frame_num, size_t frame_total);
  void startRecording();
  void stopRecording();
  void enableRenderering();

 protected slots:
  void rendererSelectedViaMenu();
};

}/* namespace visualization */
} /* namespace pcl */
#endif /* PCL_VISUALIZATION_CLOUD_PLAYER_WIDGET_H_ */
