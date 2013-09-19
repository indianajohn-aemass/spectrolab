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

#include <spectrolab/spectroscan_3d.h>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <iostream>
#include <ctime>

#define MS_DELAY 500

const int spectrolab::SpectroScan3D::IMG_RX_PORT_COMPUTER = 4002;
const int spectrolab::SpectroScan3D::CMD_RX_PORT_COMPUTER = 4000;
const int spectrolab::SpectroScan3D::CMD_TX_PORT_SCANNER = 4950;
const int spectrolab::SpectroScan3D::CMD_TX_PORT_COMPUTER = 27;

const int spectrolab::SpectroScan3D::FIRMWARE_VERSION = 0x7b;

const uint16_t spectrolab::SpectroScan3D::IMG_FRAME_DELIMITER_1 = 0xba98;
const uint16_t spectrolab::SpectroScan3D::IMG_FRAME_DELIMITER_2 = 0xfedc;

const uint32_t spectrolab::SpectroScan3D::DEFAULT_IMG_HEIGHT = 128;
const uint32_t spectrolab::SpectroScan3D::DEFAULT_IMG_WIDTH = 256;

namespace ba = boost::asio;
using std::string;

void print_debug_cout (const std::string& str);

spectrolab::SpectroScan3D::SpectroScan3D () :
    io_service_ (), io_worker_ (io_service_), cmd_timed_out_ (false),
    cmd_response_recieved_ (false), cmd_response_ (0), line_num_ (DEFAULT_IMG_HEIGHT - 1),
    running_ (false), frame_rate_ (4), frames_in_last_second_ (0),
    frame_rate_timer_ ( io_service_), current_scan_ ( new Scan (DEFAULT_IMG_HEIGHT, DEFAULT_IMG_WIDTH)),
    img_buffer_ (1024)
{
}

bool
spectrolab::SpectroScan3D::open (const boost::asio::ip::address& address)
{

  //Setup CMD Sockets
  this->cmd_rx_socket_.reset (
      new SocketT (io_service_,
          ba::ip::udp::endpoint (boost::asio::ip::udp::v4 (),
              CMD_RX_PORT_COMPUTER)));
  cmd_rx_socket_->async_receive (ba::buffer (cmd_buffer_),
      boost::bind (&SpectroScan3D::handleCMDRead, this, _1, _2));

  this->cmd_tx_socket_.reset (
      new SocketT (this->io_service_,
          boost::asio::ip::udp::endpoint (boost::asio::ip::udp::v4 (),
              CMD_TX_PORT_COMPUTER)));
  ba::ip::udp::endpoint tx_ep (address, CMD_TX_PORT_SCANNER);
  cmd_tx_socket_->connect (tx_ep);

  //IMG socket
  this->img_data_socket_.reset (
      new SocketT (io_service_,
          ba::ip::udp::endpoint (boost::asio::ip::udp::v4 (),
              IMG_RX_PORT_COMPUTER)));
  img_data_socket_->async_receive (ba::buffer (img_buffer_),
      boost::bind (&SpectroScan3D::handleImgFrame, this, _1, _2));

  io_thread_ = boost::thread (boost::bind (&SpectroScan3D::runIO, this));

  sendFirmwareCmd (RESET);
  uint8_t sys_id;
  readFirmware (SYSTEM_ID_READ, sys_id);

  std::stringstream ss;
  ss << "[SpectroScan3D]  Opened connection to scanner at  " << address
      << " with firmware version " << std::hex << (int) sys_id << " \n";
  print_debug (ss.str ());

  if (spectrolab::SpectroScan3D::FIRMWARE_VERSION != sys_id)
  {
    ss.flush ();
    ss << "[SpectroScan3D]  Warning Firmware version ( " << std::hex << sys_id
        << " is different from expected " << FIRMWARE_VERSION << " \n";
    print_debug (ss.str ());
  }

  // Set an expiration time relative to now.
  frame_rate_timer_.expires_from_now (boost::posix_time::seconds (2));
  frame_rate_timer_.async_wait (
      boost::bind (&SpectroScan3D::frameRateCB, this));
  return true;
}

void
spectrolab::SpectroScan3D::readFirmware (FirmwareReadCommands cmd,
    uint8_t& response)
{
  uint8_t buff[] = { 0, 0, 0 };
  buff[0] = cmd;
  send (buff, 3);
  response = cmd_response_.back();
}

void
spectrolab::SpectroScan3D::sendFirmwareCmd (FirmwareCommands cmd)
{
  uint8_t buff[1];
  buff[0] = cmd;
  send (buff, 1);
  Sleep(MS_DELAY);
}

bool
spectrolab::SpectroScan3D::start ()
{

  sendFirmwareCmd (RESET);
  uint8_t sys_id;
  readFirmware (SYSTEM_ID_READ, sys_id);
  if (sys_id != FIRMWARE_VERSION)
    return false;

  sendFirmwareCmd (HIGH_VOLTAGE_ON);			// 0x3A
  sendFirmwareCmd (MIRROR_SCAN_ON);				// 0x21
  sendFirmwareCmd (LASER_HIGH_POWER_ON);		// 0x1A
  sendFirmwareCmd (LASER_TEC_ON);				// 0x15
  sendFirmwareCmd (LASER_OUTPUT_ENABLE);		// 0x17
  sendFirmwareCmd (DATA_ACQUISITION_ON);		// 0x11
  sendFirmwareCmd (RX_OFFSET_VOLTAGE_REMOVE);	// 0x3F
  sendFirmwareCmd (LASER_OUTPUT_ON);			// 0x12

  running_ = true;
  proc_thread_ = boost::thread (
      boost::bind (&SpectroScan3D::processLines, this));

  return true;
}

void
spectrolab::SpectroScan3D::stop ()
{
  sendFirmwareCmd (LASER_OUTPUT_OFF);
  sendFirmwareCmd (LASER_TEC_OFF);
  sendFirmwareCmd (LASER_POWER_SUPPLY_OFF);
  sendFirmwareCmd (HIGH_VOLTAGE_OFF);
  sendFirmwareCmd (RESET);
  running_ = false;
}

void
spectrolab::SpectroScan3D::writeFirmware (FirmwareWriteCommands cmd,
    uint8_t data)
{
  uint8_t buff[2];
  buff[0] = cmd;
  buff[1] = data;
  send (buff, 2);
}

void
spectrolab::SpectroScan3D::runIO ()
{

  boost::system::error_code ec;
  io_service_.run (ec);
  if (ec)
  {
    std::stringstream ss;
    ss << "[" << boost::this_thread::get_id () << "] Error: " << ec
        << std::endl;
    std::string astring;
    ss >> astring;
    print_debug (ss.str ());
  }
}

void
spectrolab::SpectroScan3D::handleImgFrame (
    const boost::system::error_code& err, std::size_t bytes_transferred)
{

  if (err && (err != boost::asio::error::eof))
  {
    std::stringstream ss;
    ss << "[SpectroScan3D] Error reading image frame : " << err << "\n";
    print_debug (ss.str ());
    return;
  }
  boost::unique_lock<boost::mutex> lock (line_queue_mutex_);
  line_queue_.push (img_buffer_);
  line_queue_condition_.notify_all ();
  img_data_socket_->async_receive (ba::buffer (img_buffer_, 1024),
      boost::bind (&SpectroScan3D::handleImgFrame, this, _1, _2));
}

class TimeoutException : public std::exception
{
    virtual const char* what () const throw ()
    {
      return "[Spectrolab] Firmware response timed out";
    }
};

void
spectrolab::SpectroScan3D::send (uint8_t* data, size_t size)
{

  cmd_response_recieved_ = false;
  cmd_timed_out_ = false;
  cmd_tx_socket_->send (ba::buffer (data, size));
  boost::asio::deadline_timer timer (io_service_);
  timer.expires_from_now (boost::posix_time::seconds (1.5));
  timer.async_wait (boost::bind (&SpectroScan3D::handleTimeout, this, _1));
  while (!cmd_response_recieved_ && !cmd_timed_out_)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));
  }
  if (cmd_timed_out_)
  {
    cmd_timed_out_ = false;
    throw TimeoutException ();
  }
  else
    timer.cancel ();

}

bool
spectrolab::SpectroScan3D::isRunning () const
{
  return running_;
}

spectrolab::SpectroScan3D::~SpectroScan3D ()
{
  if (isRunning ())
    stop ();
  line_queue_condition_.notify_all ();
  io_service_.stop ();
  io_thread_.join ();
}

boost::signals2::connection spectrolab::SpectroScan3D::registerCallBack (
    const boost::function<sig_camera_cb>& cb)
{
  return frame_cb_.connect (cb);
}

void
spectrolab::SpectroScan3D::handleCMDRead (
    const boost::system::error_code& err, std::size_t bytes_transferred)
{

  if (err && (err != boost::asio::error::eof))
  {
    std::stringstream ss;
    ss << "[SpectroScan3D] Error reading cmd : " << err << "\n";
    print_debug (ss.str ());
    return;
  }

  cmd_response_.resize(bytes_transferred);
  for(int i=0; i<cmd_response_.size(); i++) cmd_response_[i]= cmd_buffer_[i];
  cmd_rx_socket_->async_receive (ba::buffer (cmd_buffer_),
      boost::bind (&SpectroScan3D::handleCMDRead, this, _1, _2));
  cmd_response_recieved_ = true;
}

void
spectrolab::SpectroScan3D::processLines ()
{
  while (running_)
  {
    boost::unique_lock<boost::mutex> lock (line_queue_mutex_);
    while (running_ && line_queue_.empty ())
    {
      line_queue_condition_.wait (lock);
    }
    if (!running_)
      break;
    //check for frame delimeter
    uint16_t * pixels = (uint16_t*) line_queue_.front ().data ();

    //check for a new frame
    if ( (pixels[0] == IMG_FRAME_DELIMITER_1)
        && (pixels[1] == IMG_FRAME_DELIMITER_2))
    {
      //	std::cout << "Starting new image frame " <<pixels[2] << "  " << pixels[3] << "  \n";
      line_num_ = DEFAULT_IMG_HEIGHT - 1;
    }

    if (line_num_ % 2 == 0)
    {  //laser went from right to left
      for (int i = current_scan_->cols () - 1, idx = 0; i >= 0; i--, idx += 2)
      {
        (*current_scan_) (line_num_, i).range = pixels[idx];
        (*current_scan_) (line_num_, i).amplitude = pixels[idx + 1];
      }
    }
    else
    {  //laser goes from left to right
      for (size_t i = 0, idx = 0; i < current_scan_->cols (); i++, idx += 2)
      {
        (*current_scan_) (line_num_, i).range = pixels[idx];
        (*current_scan_) (line_num_, i).amplitude = pixels[idx + 1];
      }
    }
    line_queue_.pop ();
    line_num_--;
    if (line_num_ < 0)
    {  //check to see if the frame is finished and call cb
      line_num_ = DEFAULT_IMG_HEIGHT - 1;
      frames_in_last_second_++;
      time_t scan_time;
      std::time (&scan_time);
      frame_cb_ (current_scan_, scan_time);
      current_scan_.reset (new Scan (DEFAULT_IMG_HEIGHT, DEFAULT_IMG_WIDTH));
    }
  }
}

void
spectrolab::SpectroScan3D::frameRateCB ()
{
  frame_rate_ = this->frames_in_last_second_ / 2.0f;
  frames_in_last_second_ = 0;
  frame_rate_timer_.async_wait (
      boost::bind (&SpectroScan3D::frameRateCB, this));
}

void
spectrolab::Scan::save (std::string fname) const
{
  std::ofstream ofile (fname.c_str (), std::ios::binary);

  for (int r = this->rows_ - 1; r >= 0; r--)
  {
    if (r % 2 == 0)
    {  //laser went from right to left
      for (int c = columns_ - 1; c >= 0; c--)
      {
        ofile.write ((char*) & (*this) (r, c).amplitude, 2);
        ofile.write ((char*) & (*this) (r, c).range, 2);
      }
    }
    else
    {  //laser goes from left to right
      for (int c = 0; c < columns_; c++)
      {
        ofile.write ((char*) & (*this) (r, c).amplitude, 2);
        ofile.write ((char*) & (*this) (r, c).range, 2);
      }
    }
  }
  //add in size information to the file
  ofile.seekp (0, std::ios::beg);
  ofile.write ((char*) &rows_, 2);
  ofile.write ((char*) &columns_, 2);

}

bool
spectrolab::Scan::load (std::string fname)
{

  std::ifstream ifile (fname.c_str (), std::ios::binary);
  if (!ifile.is_open ())
    return false;

  ifile.read ((char*) &rows_, 2);
  ifile.read ((char*) &columns_, 2);

  //detect if it was the old file format with no size information
  //ordering is swapped because of byte swap operation from original spectrolab file implementation
  if ( (rows_ == SpectroScan3D::IMG_FRAME_DELIMITER_2)
      && (columns_ == SpectroScan3D::IMG_FRAME_DELIMITER_1))
  {
    rows_ = SpectroScan3D::DEFAULT_IMG_HEIGHT;
    columns_ = SpectroScan3D::DEFAULT_IMG_WIDTH;
  }
  resize (rows_, columns_);
  //restart to beginning
  ifile.seekg (0, std::ios::beg);

  for (int r = this->rows_ - 1; r >= 0; r--)
  {
    if (r % 2 == 0)
    {  //laser went from right to left
      for (int c = columns_ - 1; c >= 0; c--)
      {
        ifile.read ((char*) & (*this) (r, c).amplitude, 2);
        ifile.read ((char*) & (*this) (r, c).range, 2);
      }
    }
    else
    {  //laser goes from left to right
      for (int c = 0; c < columns_; c++)
      {
        ifile.read ((char*) & (*this) (r, c).amplitude, 2);
        ifile.read ((char*) & (*this) (r, c).range, 2);
      }
    }
  }


  //to keep the frame format consistent with the communication protocol,
  //put back the delimeter tags
  Pixel& delimeter_pixel = (*this)(rows_-1,0);
  delimeter_pixel.range = SpectroScan3D::IMG_FRAME_DELIMITER_1;
  delimeter_pixel.amplitude = SpectroScan3D::IMG_FRAME_DELIMITER_2;
  return true;
}

void
print_debug_cout (const std::string& str)
{
  std::cout << str;
}
