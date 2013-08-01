/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012  Spectrolab
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

#ifndef _PCL_SPECTROSCAN_3D_IO_H_
#define _PCL_SPECTROSCAN_3D_IO_H_

#include <spectrolab/spectroscan_3d.h>
#include <pcl/io/grabber.h>

#include <pcl/io/movie_grabber.h>
#include <pcl/io/recorder.h>

namespace pcl {

struct PointXYZ;
struct PointXYZI;
template<typename T> class PointCloud;

/** \brief Settings for projecting Spectroscan3D imgs to Point Clouds
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup io
 */
struct SpectroscanSettings {
  double range_resolution;
  double range_offset;
  double x_angle_delta;  //radians
  double y_angle_delta;  //radians
  double min_range;
  double max_range;
  SpectroscanSettings();
  bool load(std::string fname);
  void save(std::string ofname);
};

/** \brief Converts a Spectrolab scan into a PCL Point Cloud.
 */

void rangeImageToCloud(const spectrolab::Scan& scan,
                       pcl::PointCloud<pcl::PointXYZI>& cloud,
                       const SpectroscanSettings& settings);

/** \brief Grabber for the Spectrolab Lidar Camera
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup io
 */

class PCL_EXPORTS Spectroscan3DGrabber : public Grabber {
 public:
  Spectroscan3DGrabber(std::string ipaddress = "192.168.0.27");

  virtual ~Spectroscan3DGrabber() throw () {
  }

  //define callback signature typedefs
  typedef void (sig_cb_cloud)(
      const boost::shared_ptr<const sensor_msgs::PointCloud2>&);
  typedef void (sig_cb_xyz_cloud)(
      const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
  typedef void (sig_cb_xyzi_cloud)(
      const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);

  /** \brief For devices that are streaming, the streams are started by calling this method.
   *        Trigger-based devices, just trigger the device once for each call of start.
   */
  virtual void
  start();

  /** \brief For devices that are streaming, the streams are stopped.
   *        This method has no effect for triggered devices.
   */
  virtual void
  stop();

  /** \brief returns the name of the concrete subclass.
   * \return the name of the concrete driver.
   */
  virtual std::string getName() const {
    return "spectroscan3d";
  }

  /** \brief Indicates whether the grabber is streaming or not. This value is not defined for triggered devices.
   * \return true if grabber is running / streaming. False otherwise.
   */
  virtual bool isRunning() const {
    return camera_.isRunning();
  }

  /** \brief returns fps. 0 if trigger based. */
  virtual float getFramesPerSecond() const {
    return camera_.getFrameRate();
  }

  /** \brief Set Spectrolab settings for grabber*/
  void setSettings(const SpectroscanSettings& settings) {
    settings_ = settings;
  }

  /*
   * Spectrolab requested hook for performing filtering on the
   * PointCloud before it is broadcasted to all registered callbacks
   */
  typedef void (FilterFunctT)(
      const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >&,
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >&);

  /** \brief sets the filter used before publishing the point cloud
   */
  void setFilter(const FilterFunctT& funct) {
    filter_ = funct;
  }

  /** \brief sets the filter used before publishing the point cloud
   */
  spectrolab::SpectroScan3D&
  getDriver() {
    return camera_;
  }

 private:
  spectrolab::SpectroScan3D camera_;
  SpectroscanSettings settings_;

  boost::function<FilterFunctT> filter_;

  boost::signals2::signal<spectrolab::SpectroScan3D::sig_camera_cb>* img_cb_;
  boost::signals2::signal<sig_cb_xyz_cloud>* xyz_cb_;
  boost::signals2::signal<sig_cb_xyzi_cloud>* xyzi_cb_;
  boost::signals2::signal<sig_cb_cloud>* cloud_cb_;

  void frameCB(const spectrolab::Scan::ConstPtr& scan, time_t scan_time);

 protected:
  virtual void
  signalsChanged();
};

/** \brief Grabber for replaying a directory of Spectrolab lidar frames
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup io
 */

class PCL_EXPORTS Spectroscan3DMovieGrabber : public MovieGrabber {
 public:
  Spectroscan3DMovieGrabber(boost::filesystem::path movie_dir,
                            bool use_extention = false);

  void setSettings(const SpectroscanSettings& settings) {
    settings_ = settings;
  }

 protected:

  virtual void
  handleFile(const std::string& file);

  boost::signals2::signal<spectrolab::SpectroScan3D::sig_camera_cb>* img_cb_;
  SpectroscanSettings settings_;
};

/** \brief Recorder for saving streams of Spectrolab lidar frames
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup io
 */
class PCL_EXPORTS Spectroscan3DRecorder : public Recorder {
 public:
  Spectroscan3DRecorder();

  virtual bool
  setGrabber(const boost::shared_ptr<Grabber>& grabber);

  virtual bool
  hasValidGrabber();

  virtual bool
  isRecording();

  virtual void
  start();

  virtual void
  stop();

 protected:
  boost::shared_ptr<Grabber> grabber_;

  boost::signals2::connection connection_;

  bool valid_grabber_;

  void
  frameCB(const spectrolab::Scan::ConstPtr&, time_t);
};
}

#endif /* SPECTROLAB_IO_H_ */
