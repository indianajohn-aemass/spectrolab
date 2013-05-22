/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012 The MITRE Corporation
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

#ifndef _SPECTROLAB_SPECTROSCAN_3D_H_
#define _SPECTROLAB_SPECTROSCAN_3D_H_

#include <boost/multi_array.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>

namespace spectrolab{


	/*
	 * RangeImage
	 * Range Image in meters for the camera
	 * NaN values are invalid points
	 */
	typedef boost::multi_array<float, 2> RangeImage;
	typedef boost::shared_ptr<RangeImage> RangeImgaePtr;

	typedef boost::multi_array<float, 2> IntensityImage;
	typedef boost::shared_ptr<IntensityImage> IntensityImagePtr;


	  /** \brief Grabber for the Spectrolab Lidar Camera
	   * \author Adam Stambler <adasta@gmail.com>
	   * \ingroup io
	   *
	   *
	   *  command read and writes are synchronous
	   *  image data is asynchronous
	   *
	   */
	class SpectroScan3D{
		public:

		SpectroScan3D(const boost::asio::ip::address& ipAddress=
				boost::asio::ip::address::from_string("192.168.0.27"));
		//define callback signature typedefs
		typedef void (sig_camera_cb) (const RangeImgaePtr&, const IntensityImagePtr&);

		bool open(const boost::asio::ip::address& ipAddress);

		bool isRunning();
		bool start();
		void stop();

		boost::signals2::connection
		regsiterCallBack(const boost::function<sig_camera_cb>& cb);

		enum FirmwareCommands{
			//commands with no real response
			// camera only returns 0x00
			RESET=0x10,
			MIRROR_SCAN_ON=0x21,
			MIRROR_SCAN_OFF=0x22,
			LASER_HIGH_POWER_ON=0x1A,
			LASER_TEC_ON=0x15,
			LASER_OUTPUT_ENABLE=0x17,
			LASER_OUTPUT_ON=0x12,
			DATA_ACQUISITION_ON=0x11,
			LASER_OUTPUT_OFF=0x18,
			LASER_TEC_OFF=0x16,
			LASER_POWER_SUPPLY_OFF=0x13,
			LASER_LOW_POWER_ON=0x19,
			LED1_TOGGLE=0x1F,
			LASER_STATUS_DISPLAY=0x2D,
			SYSTEM_STATUS_DISPLAY=0x2E,
			INTERPOLATOR_ON=0x31,
			INTERPOLATOR_OFF=0x39,
			ADC_CALIBRATE=0x36,
			REFERENCE_PULSE_ON=0x37,
			REFERENCE_PULSE_OFF=0x38,
			HIGH_VOLTAGE_ON=0x3A,
			HIGH_VOLTAGE_OFF=0x3B,
			RX_OFFSET_VOLTAGE_REMOVE=0x3F,
		};


		enum FirmwareReadCommands{
			//sending padds each command with 0x00 0x00
			//return is CMD 0x00 0xNN
			LASER_STATUS_READ=0x25,
			SYSTEM_STATUS_READ=0x26,
			SYSTEM_ID_READ=0x27,
			IN_FIFO_DELAY_READ=0x30
		};

		enum FirmwareWriteCommands{
			//write command is CMD 0xNN
			//response is  0x00 0x00
			ODD_SYNC_DELAY_WRITE=0x23,
			EVEN_SYNC_DELAY_WRITE=0x2F,
			HIGH_GAIN_AMPLITUDE_TRHESHOLD_WRITE=0x24,
			HIGH_GAIN_AMPLITUDE_SATURATE_WRITE=0x34,
			LOW_GAIN_AMPLITUDE_THRESHOLD_WRITE=0x28,
			LOW_GAIN_AMPLITUDE_SATURATE_WRITE=0x35,
			IN_FIFO_DELAY_WRITE=0x2A,
			TARGET_SELECT_WRITE=0x33,
		};

		void readFirmware( FirmwareReadCommands cmd, uint8_t& response);

		void sendFirmwareCmd(FirmwareCommands cmd);

		void writeFirmware(FirmwareWriteCommands cmd, uint8_t data);

		private:
			int range_img_width_;  //256
			int range_img_height_; //128

		  boost::asio::streambuf img_buffer_;

	      boost::thread grabber_thread_;
	      typedef boost::asio::ip::udp::socket SocketT;

	      boost::shared_ptr<SocketT> img_data_socket_;
	      boost::shared_ptr<SocketT> cmd_socket_;

	      boost::asio::io_service img_io_service_;
	      boost::asio::io_service cmd_io_service_;

	      static const int IMAGE_DATA_PORT;
	      static const int COMMAND_PORT;
	      static const uint16_t IMG_FRAME_DELIMITER_B1; //Image frame delimiter byte 1
	      static const uint16_t IMG_FRAME_DELIMITER_B2; //Image frame delimiter byte 2

	      void handleImgFrame( const boost::system::error_code& err);

	};
}


#endif /* LIDAR_DRIVER_H_ */
