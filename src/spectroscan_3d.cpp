/*
 * lidar_driver.cpp
 *
 *  Created on: May 5, 2013
 *      Author: asher
 */

#include <spectrolab/spectroscan_3d.h>

const int spectrolab::SpectroScan3D::IMG_RX_PORT_COMPUTER=4002;
const int spectrolab::SpectroScan3D::CMD_RX_PORT_COMPUTER=4000;
const int spectrolab::SpectroScan3D::CMD_TX_PORT_SCANNER=4950;
const int spectrolab::SpectroScan3D::CMD_TX_PORT_COMPUTER=27;

const int spectrolab::SpectroScan3D::FIRMWARE_VERSION=0x7b;

const uint16_t spectrolab::SpectroScan3D::IMG_FRAME_DELIMITER_1=0xba98;
const uint16_t spectrolab::SpectroScan3D::IMG_FRAME_DELIMITER_2=0xfedc;

namespace ba=boost::asio;

 spectrolab::SpectroScan3D::SpectroScan3D(const boost::asio::ip::address& address) :
		 io_service_(), io_worker_(io_service_),
		 cmd_timed_out_(false), cmd_response_recieved_(false), cmd_response_(0),
		 line_num_(0), running_(false),
		 range_img_(new uint16_t[256*128], boost::extents[128][256]),
		 intensity_img_(new uint16_t[256*128] , boost::extents[128][256])
		 {
	 this->open(address);
 }

 bool
 spectrolab::SpectroScan3D::open(const boost::asio::ip::address& address){

	std::cout << "[SpectroScan3D] Connecting to IP address  : " << address << " \n";

	//Setup CMD Sockets
	this->cmd_rx_socket_.reset(new SocketT(io_service_,
			 ba::ip::udp::endpoint(boost::asio::ip::udp::v4(), CMD_RX_PORT_COMPUTER) ) );
	cmd_rx_socket_->async_receive( ba::buffer(cmd_buffer_), boost::bind(&SpectroScan3D::handleCMDRead, this, _1, _2));

	this->cmd_tx_socket_.reset(new SocketT(this->io_service_,
		 boost::asio::ip::udp::endpoint(  boost::asio::ip::udp::v4(), CMD_TX_PORT_COMPUTER) ));
	ba::ip::udp::endpoint tx_ep(address, CMD_TX_PORT_SCANNER);
	cmd_tx_socket_->connect(tx_ep);

	//IMG socket
	this->img_data_socket_.reset(new SocketT(io_service_,
			 ba::ip::udp::endpoint(boost::asio::ip::udp::v4(), IMG_RX_PORT_COMPUTER)));
	img_data_socket_->async_receive(ba::buffer(img_buffer_), boost::bind(&SpectroScan3D::handleImgFrame, this, _1, _2));

	io_thread_ = boost::thread( boost::bind(&SpectroScan3D::runIO, this));

	uint8_t sys_id;
	readFirmware(SYSTEM_ID_READ, sys_id);
	std::cout << "Opened connection to scanner at  " << address << " with firmware version " << std::hex <<   (int) sys_id  << " \n";

	sendFirmwareCmd(RESET);
	return true;
 }


void spectrolab::SpectroScan3D::readFirmware(FirmwareReadCommands cmd,
		uint8_t& response) {
	uint8_t buff[]= {0, 0,0};
	buff[0] = cmd;
	send(buff,3);
	response = cmd_response_;
}

void spectrolab::SpectroScan3D::sendFirmwareCmd(FirmwareCommands cmd) {
	uint8_t buff[1];
	buff[0]= cmd;
	send(buff,1);
}

bool spectrolab::SpectroScan3D::start() {

	sendFirmwareCmd(RESET);
	uint8_t sys_id;
	readFirmware(SYSTEM_ID_READ, sys_id);
	if (sys_id != FIRMWARE_VERSION) return false;

	sendFirmwareCmd(HIGH_VOLTAGE_ON);
	sendFirmwareCmd(MIRROR_SCAN_ON);
	sendFirmwareCmd(LASER_HIGH_POWER_ON);
	sendFirmwareCmd(LASER_TEC_ON);
	sendFirmwareCmd(LASER_OUTPUT_ENABLE);
	sendFirmwareCmd(RX_OFFSET_VOLTAGE_REMOVE);
	sendFirmwareCmd(LASER_OUTPUT_ON);
	sendFirmwareCmd(DATA_ACQUISITION_ON);

	running_ =true;
	return true;
}

void spectrolab::SpectroScan3D::stop() {
	sendFirmwareCmd(LASER_OUTPUT_OFF);
	sendFirmwareCmd(LASER_TEC_OFF);
	sendFirmwareCmd(LASER_POWER_SUPPLY_OFF);
	sendFirmwareCmd(HIGH_VOLTAGE_OFF);
	sendFirmwareCmd(RESET);
	running_=false;
}

void spectrolab::SpectroScan3D::writeFirmware(FirmwareWriteCommands cmd,
		uint8_t data) {
	uint8_t buff[2];
	buff[0]= cmd;
	buff[1]= data;
	send(buff,2);
}

void spectrolab::SpectroScan3D::runIO() {

		boost::system::error_code ec;
		io_service_.run(ec) ;
		if( ec )
		{
			std::stringstream ss;
			ss  << "[" << boost::this_thread::get_id() << "] Error: " << ec << std::endl;
			std::string astring;
			ss >> astring;
			std::cout << ss;
		}
}

void spectrolab::SpectroScan3D::handleImgFrame(const boost::system::error_code& err,  std::size_t bytes_transferred ) {

	if (err && (err != boost::asio::error::eof) )
    {
      std::cout << "[SpectroScan3D] Error reading image frame : " << err << "\n";
      return;
    }

	//check for frame delimeter
	uint16_t  * pixels = (uint16_t*) img_buffer_;

	if ( (pixels[0] ==  IMG_FRAME_DELIMITER_1) &&
	     (pixels[1] ==  IMG_FRAME_DELIMITER_2)){
		std::cout << "Starting new image frame " <<pixels[2] << "  " << pixels[3] << " \n";
		line_num_=0;
	}

	for(size_t i=0, idx=0; i< range_img_.shape()[1]; i++, idx+=2){
		range_img_[line_num_][i] = pixels[idx+1];
		intensity_img_[line_num_][i] = pixels[idx];
	}

	line_num_++;
 	if (line_num_ >= range_img_.shape()[0]){
		if (frame_cb_.num_slots()) frame_cb_(range_img_, intensity_img_);
		line_num_=0;
	}

	img_data_socket_->async_receive(ba::buffer(img_buffer_), boost::bind(&SpectroScan3D::handleImgFrame, this, _1, _2));
}


class TimeoutException: public std::exception
{
  virtual const char* what() const throw()
  {
    return "[Spectrolab] Firmware response timed out";
  }
};

void spectrolab::SpectroScan3D::send(uint8_t* data, size_t size) {

	cmd_response_recieved_ =false;
	cmd_timed_out_ =false;
	cmd_tx_socket_->send(ba::buffer(data, size));
	boost::asio::deadline_timer timer( io_service_ );
	timer.expires_from_now( boost::posix_time::seconds( 1) );
	timer.async_wait( boost::bind(&SpectroScan3D::handleTimeout, this,_1) );
	while(!cmd_response_recieved_  && !cmd_timed_out_){
		usleep(1);
	}
	if (cmd_timed_out_){
		cmd_timed_out_ = false;
		throw TimeoutException();
	}
	else timer.cancel();
}

bool spectrolab::SpectroScan3D::isRunning() {
	return running_;
}

spectrolab::SpectroScan3D::~SpectroScan3D() {
	if (isRunning()) stop();
	io_service_.stop();
	io_thread_.join();
	delete range_img_.data();
	delete intensity_img_.data();
}

boost::signals2::connection spectrolab::SpectroScan3D::regsiterCallBack(
		const boost::function<sig_camera_cb>& cb) {
	return frame_cb_.connect(cb);
}

void spectrolab::SpectroScan3D::handleCMDRead(const boost::system::error_code& err,  std::size_t bytes_transferred ) {

	if (err && (err != boost::asio::error::eof) )
    {
      std::cout << "[SpectroScan3D] Error reading cmd : " << err << "\n";
      return;
    }
/*	//parse response
	std::cout << "received " ;
	for(int i=0; i<bytes_transferred; i++){
		std::cout << std::hex << (int) cmd_buffer_[i] << " ";
	}
	std::cout << " \n";
	*/
	cmd_response_ = cmd_buffer_[ bytes_transferred-1];
	cmd_rx_socket_->async_receive( ba::buffer(cmd_buffer_), boost::bind(&SpectroScan3D::handleCMDRead, this, _1, _2));
	cmd_response_recieved_ =true;
}

