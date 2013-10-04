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
#include <pcl/io/spectroscan_3d_io.h>
#include <spectrolab/spectroscan_3d.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/console/print.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/io.h>
#include <pcl/exceptions.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>

#define OVERSCAN 64	//rm # of total overscan pixels (non-image) for a row
#define X_SCAN_AMPL 20 // rm amplitude of sinusoidal scan pattern in the x-axis (in degrees)

pcl::SpectroscanSettings::SpectroscanSettings () :
    range_resolution (0.00625), range_offset (0), x_angle_delta (
        0.15625 / 180 * M_PI), y_angle_delta (0.160 / 180 * M_PI), min_range (0),
        max_range (20)
{
}

bool
pcl::SpectroscanSettings::load (std::string fname)
{
  using boost::property_tree::ptree;
  ptree pt;

  double t_max_range, t_min_range, t_range_offset, t_y_ange_delta,
      t_x_ange_delta;

  // Load the ini file into the property tree. If reading fails
  // (cannot open file, parse error), an exception is thrown.
  try
  {
    read_ini (fname, pt);
    t_max_range = pt.get<double> ("max_range");
    t_min_range = pt.get<double> ("min_range");
    t_range_offset = pt.get<double> ("range_offset");
    t_y_ange_delta = pt.get<double> ("y_angle_delta");
    t_x_ange_delta = pt.get<double> ("x_angle_delta");
  }
  catch (std::exception& e)
  {
    pcl::console::print_error ("[SpectroscanSettings::load] Error : %s",
        e.what ());
    return false;
  }
  range_offset = t_range_offset;
  max_range = t_max_range;
  min_range = t_min_range;
  x_angle_delta = t_x_ange_delta;
  y_angle_delta = t_y_ange_delta;
  return true;
}

void
pcl::SpectroscanSettings::save (std::string ofname)
{
  boost::filesystem::path opath (ofname);
  opath.replace_extension (".ini");
  // Create an empty property tree object
  using boost::property_tree::ptree;
  ptree pt;

  pt.put ("max_range", max_range);
  pt.put ("min_range", min_range);
  pt.put ("range_offset", range_offset);
  pt.put ("y_angle_delta", y_angle_delta);
  pt.put ("x_angle_delta", x_angle_delta);

  // Write the property tree to the XML file.
  write_ini (opath.string (), pt);
}

void
pcl::rangeImageToCloud (const spectrolab::Scan& scan,
    pcl::PointCloud<pcl::PointXYZI>& cloud, const SpectroscanSettings& settings)
{

  cloud.resize (scan.cols () * scan.rows ());
  cloud.width = scan.cols ();
  cloud.height = scan.rows ();
  cloud.sensor_origin_ << 0, 0, 0, 1;
  cloud.sensor_orientation_ = Eigen::Quaternionf::Identity ();
  cloud.is_dense = false;

  float mx = scan.cols () / 2.0f;
  float my = scan.rows () / 2.0f;
  float A = X_SCAN_AMPL/(sin(M_PI*mx/(2*mx + OVERSCAN)))*M_PI/180;	// rm (in radians)

  for (size_t r = 0, idx = 0; r < scan.rows (); r++)
  {
    for (size_t c = 0; c < scan.cols (); c++, idx++)
    {
      double range = (float) scan[idx].range;  //scan(r,c).range ;
      range = settings.range_resolution * range + settings.range_offset;
      if ( (range > settings.max_range) || (range < settings.min_range))
      {
        cloud[idx].x = cloud[idx].y, cloud[idx].z =
            std::numeric_limits<float>::quiet_NaN ();
        cloud[idx].intensity = 0;
        continue;
      }
      cloud[idx].z = range;
      //float dx = c - mx;
	  float x_angle = A*sin(M_PI*(c - mx)/(scan.cols () + OVERSCAN));	// rm (in radians)
      //cloud[idx].x = sin (dx * settings.x_angle_delta) * range;
	  cloud[idx].x = sin (x_angle) * range;		// rm
      float dy = r - my;
      cloud[idx].y = sin (dy * settings.y_angle_delta) * range;

      float amp = ((float) scan[idx].amplitude);
      cloud[idx].intensity = amp / 1024.0f;
    }
  }
  pcl::PointXYZI& pt_delimeter = cloud.at (0, scan.rows () - 1);
  pcl::PointXYZI& pt_frame_count = cloud.at (1, scan.rows () - 1);
  pt_delimeter.x = pt_delimeter.y = pt_delimeter.z = pt_frame_count.x =
      pt_frame_count.y, pt_frame_count.z =
      std::numeric_limits<float>::quiet_NaN ();
  pt_delimeter.intensity = 0;
  pt_frame_count.intensity = 0;
}

void
print_debug_output (const std::string& str)
{
  pcl::console::print_debug ("%s", str.c_str ());
}

void
passthrough_filter (
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >& in,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >& out)
{
  out = in;
}

pcl::Spectroscan3DGrabber::Spectroscan3DGrabber (std::string ipaddress)
{

  camera_.setDebugOutput (print_debug_output);
  if (!camera_.open (boost::asio::ip::address::from_string (ipaddress)))
  {
    throw pcl::IOException ("Failed to open Spectroscan 3D Camera");
  }
  camera_.registerCallBack (
      boost::bind (&Spectroscan3DGrabber::frameCB, this, _1, _2));
  xyzi_cb_ = this->createSignal<sig_cb_xyzi_cloud> ();
  img_cb_ = this->createSignal<spectrolab::SpectroScan3D::sig_camera_cb> ();
  xyz_cb_ = this->createSignal<sig_cb_xyz_cloud> ();
  cloud_cb_ = this->createSignal<sig_cb_cloud> ();
  filter_ = passthrough_filter;
}

void
pcl::Spectroscan3DGrabber::start ()
{
  camera_.start ();
}

void
pcl::Spectroscan3DGrabber::stop ()
{
  camera_.stop ();
}

void
pcl::Spectroscan3DGrabber::frameCB (const spectrolab::Scan::ConstPtr& scan,
    time_t scan_time)
{
  if (!img_cb_->empty ())
  {
    (*img_cb_) (scan, scan_time);
  }
  PointCloud<pcl::PointXYZI>::Ptr xyzi (new pcl::PointCloud<pcl::PointXYZI>);
  PointCloud<pcl::PointXYZI>::Ptr filtered_xyzi (
      new pcl::PointCloud<pcl::PointXYZI>);

  rangeImageToCloud (*scan, *xyzi, settings_);

  filter_ (xyzi, filtered_xyzi);

  if (!xyzi_cb_->empty ())
  {
    (*xyzi_cb_) (filtered_xyzi);
  }
  if (!xyz_cb_->empty ())
  {
    PointCloud<pcl::PointXYZ>::Ptr xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (*filtered_xyzi, *xyz);
    (*xyz_cb_) (xyz);
  }
  if (!cloud_cb_->empty ())
  {
    sensor_msgs::PointCloud2Ptr cloud (new sensor_msgs::PointCloud2);
    pcl::toROSMsg (*filtered_xyzi, *cloud);
    (*cloud_cb_) (cloud);
  }
}

void
pcl::Spectroscan3DGrabber::signalsChanged ()
{
  cloud_cb_ = this->find_signal<sig_cb_cloud> ();
  xyz_cb_ = this->find_signal<sig_cb_xyz_cloud> ();
  xyzi_cb_ = this->find_signal<sig_cb_xyzi_cloud> ();
  img_cb_ = this->find_signal<spectrolab::SpectroScan3D::sig_camera_cb> ();
}

pcl::Spectroscan3DMovieGrabber::Spectroscan3DMovieGrabber (
    boost::filesystem::path movie_dir, bool use_extention) :
    MovieGrabber (movie_dir, use_extention ? ".ssi" : ""), img_cb_ (NULL)
{
  img_cb_ = this->createSignal<spectrolab::SpectroScan3D::sig_camera_cb> ();
}

void
pcl::Spectroscan3DMovieGrabber::handleFile (const std::string& file)
{

  spectrolab::Scan::Ptr scan (new spectrolab::Scan ());
  if (!scan->load (file))
  {
    pcl::console::print_error ("[Spectroscan3DMovieGrabber] Cannot read %s",
        file.c_str ());
    return;
  }
  if (!img_cb_->empty ())
  {
    (*img_cb_) (scan, getCurrentFrame ());
  }
  PointCloud<pcl::PointXYZI>::Ptr xyzi (new pcl::PointCloud<pcl::PointXYZI>);
  rangeImageToCloud (*scan, *xyzi, settings_);

  sensor_msgs::PointCloud2Ptr cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg<pcl::PointXYZI> (*xyzi, *cloud);
  handleCloud (cloud, Eigen::Vector4f (0, 0, 0, 1),
      Eigen::Quaternionf::Identity ());
}

pcl::Spectroscan3DRecorder::Spectroscan3DRecorder () :
    Recorder ("Record to Spectroscan 3D Frames"), valid_grabber_ (false)
{

}

bool
pcl::Spectroscan3DRecorder::setGrabber (
    const boost::shared_ptr<Grabber>& grabber)
{
  valid_grabber_ = grabber->providesCallback<
      spectrolab::SpectroScan3D::sig_camera_cb> ();
  grabber_ = grabber;
  return valid_grabber_;
}

bool
pcl::Spectroscan3DRecorder::hasValidGrabber ()
{
  return valid_grabber_;
}

bool
pcl::Spectroscan3DRecorder::isRecording ()
{
  return connection_.connected ();
}

void
pcl::Spectroscan3DRecorder::start ()
{
  if (!valid_grabber_)
    return;
  connection_ = grabber_->registerCallback<
      spectrolab::SpectroScan3D::sig_camera_cb> (
      boost::bind (&Spectroscan3DRecorder::frameCB, this, _1, _2));
}

void
pcl::Spectroscan3DRecorder::stop ()
{
  connection_.disconnect ();
}

void
pcl::Spectroscan3DRecorder::frameCB (
    const spectrolab::Scan::ConstPtr& scan, time_t t)
{
  scan->save (genNextFileName () + ".ssi");
}

