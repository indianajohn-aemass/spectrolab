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
#include <pcl/io/movie_grabber.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

pcl::MovieGrabber::MovieGrabber (const boost::filesystem::path movie_folder,
    std::string ext) :
    running_ (false), frame_idx_ (0)
{
  setFramesPerSecond (30);

  cloud_cb_ = this->createSignal<sig_pointcloud_cb> ();
  xyz_cb_ = this->createSignal<sig_xyz_cb> ();
  xyzi_cb_ = this->createSignal<sig_xyzi_cb> ();
  xyzrgb_cb_ = this->createSignal<sig_xyzrgb_cb> ();
  frame_num_cb_ = this->createSignal<sig_frame_num_cb> ();

  if (!boost::filesystem::exists (movie_folder))
  {
    pcl::console::print_error (
        "[MovieGrabber] There is no file/movie folder at %s",
        movie_folder.c_str ());
    return;
  }

  if (boost::filesystem::is_regular_file (movie_folder)
      && (movie_folder.extension ().string () == ext))
  {
    file_names_.push_back (movie_folder.string ());
  }
  else
  {
    boost::filesystem::directory_iterator diter (movie_folder), dend;
    for (; diter != dend; diter++)
    {
      if ( (diter->path ().extension () == ext)
          && (boost::filesystem::is_regular_file (diter->path ())))
        file_names_.push_back (diter->path ().string ());
    }
    std::sort (file_names_.begin (), file_names_.end ());
  }
}

void
pcl::MovieGrabber::start ()
{
  if (running_)
    return;
  running_ = true;
  io_thread_ = boost::thread (boost::bind (&MovieGrabber::runIO, this));
}

void
pcl::MovieGrabber::stop ()
{
  running_ = false;
  this->io_thread_.join ();
}

bool
pcl::MovieGrabber::isRunning () const
{
  return running_;
}

float
pcl::MovieGrabber::getFramesPerSecond () const
{
  return frame_rate_;
}

void
pcl::MovieGrabber::setFramesPerSecond (float frame_rate)
{
  frame_rate_ = frame_rate;
  sleep_ms_ = 1.0f / frame_rate * 1000;
}

size_t
pcl::MovieGrabber::getFrameCount ()
{
  return this->file_names_.size ();
}

void
pcl::MovieGrabber::setFrameNumber (size_t frame_number)
{
  boost::unique_lock<boost::mutex> lock (frame_mutex_);
  if (frame_number >= this->file_names_.size ())
    return;
  frame_idx_ = frame_number;
}

size_t
pcl::MovieGrabber::getCurrentFrame ()
{
  return frame_idx_;
}

std::string
pcl::MovieGrabber::getMovieDir ()
{
  return movie_dir_.string ();
}

void
pcl::MovieGrabber::runIO ()
{
  while ( (frame_idx_ < file_names_.size ()) && running_)
  {
    handleFile (file_names_[frame_idx_]);
    (*frame_num_cb_) (frame_idx_, file_names_.size ());
    boost::this_thread::sleep (boost::posix_time::milliseconds (sleep_ms_));
    boost::unique_lock<boost::mutex> lock (frame_mutex_);
    frame_idx_++;
  }
}

void
pcl::MovieGrabber::handleFile (const std::string& file)
{
  sensor_msgs::PointCloud2Ptr cloud (new sensor_msgs::PointCloud2);
  Eigen::Vector4f origin;
  Eigen::Quaternionf rot;
  if (pcl::io::loadPCDFile (file, *cloud, origin, rot) < 0)
  {
    pcl::console::print_error ("[MovieGrabber] Failed to load frame %s",
        file.c_str ());
    return;
  }
  handleCloud (cloud, origin, rot);
}

void
pcl::MovieGrabber::playOneFrame ()
{
  handleFile (file_names_[frame_idx_]);
  frame_idx_++;
}

void
pcl::MovieGrabber::handleCloud (
    const sensor_msgs::PointCloud2ConstPtr& cloud,
    const Eigen::Vector4f& origin, const Eigen::Quaternionf& rot)
{
  if (!cloud_cb_->empty ())
    (*cloud_cb_) (cloud);
  if (!xyz_cb_->empty ())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud (
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud, *tcloud);
    tcloud->sensor_orientation_ = rot;
    tcloud->sensor_origin_ = origin;
    (*xyz_cb_) (tcloud);
  }
  if (!xyzi_cb_->empty ())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr tcloud (
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*cloud, *tcloud);
    tcloud->sensor_orientation_ = rot;
    tcloud->sensor_origin_ = origin;
    (*xyzi_cb_) (tcloud);
  }
  if (!xyzrgb_cb_->empty ())
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tcloud (
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*cloud, *tcloud);
    tcloud->sensor_orientation_ = rot;
    tcloud->sensor_origin_ = origin;
    (*xyzrgb_cb_) (tcloud);
  }
}
