/*
 * lidar_driver.cpp
 *
 *  Created on: May 5, 2013
 *      Author: asher
 */

#include <spectrolab/spectroscan_3d.h>

const int spectrolab::SpectroScan3D::IMAGE_DATA_PORT=4002;
const int spectrolab::SpectroScan3D::COMMAND_PORT=4000;
const uint16_t spectrolab::SpectroScan3D::IMG_FRAME_DELIMITER_B1=0xFEDC;
const uint16_t spectrolab::SpectroScan3D::IMG_FRAME_DELIMITER_B2=0xBA98;

namespace ba=boost::asio;

 spectrolab::SpectroScan3D::SpectroScan3D(const boost::asio::ip::address& address):
	 range_img_height_(128), range_img_width_(256){

	 this->open(address);
 }

 bool
 spectrolab::SpectroScan3D::open(const boost::asio::ip::address& address){

	 std::cout << "[SpectroScan3D] Connecting to IP address  : " << address << " \n";
	 ba::ip::udp::endpoint cmd_endpoint(address, COMMAND_PORT);
	 this->cmd_socket_.reset(new SocketT(this->cmd_io_service_));
	 this->cmd_socket_->connect(cmd_endpoint);
	 if (!cmd_socket_->is_open())  return false;

	sendFirmwareCmd(RESET);
	uint8_t sys_id;
	readFirmware(SYSTEM_ID_READ, sys_id);
	if (sys_id != 0x6b) return false;

	ba::ip::udp::endpoint img_endpoint(address, IMAGE_DATA_PORT);
	this->img_data_socket_.reset(new SocketT(this->img_io_service_));
	this->img_data_socket_->connect(img_endpoint);
	if (!img_data_socket_->is_open())  return false;

	//TODO setup image data callbacks
	//ba::async_read_until(*img_data_socket_, img_buffer_, )
	return true;
 }


void spectrolab::SpectroScan3D::readFirmware(FirmwareReadCommands cmd,
		uint8_t& response) {

}

void spectrolab::SpectroScan3D::sendFirmwareCmd(FirmwareCommands cmd) {
	uint8_t buff[10];
	buff[0]= cmd;
	buff[1]=255;
	cmd_socket_->send(ba::buffer(buff, 2));
	cmd_socket_->receive(ba::buffer(buff,2));
	assert(buff[0]==0);
}


bool spectrolab::SpectroScan3D::start() {

	sendFirmwareCmd(RESET);
	uint8_t sys_id;
	readFirmware(SYSTEM_ID_READ, sys_id);
	if (sys_id != 0x6b) return false;

	sendFirmwareCmd(HIGH_VOLTAGE_ON);
	sendFirmwareCmd(MIRROR_SCAN_ON);
	sendFirmwareCmd(LASER_HIGH_POWER_ON);
	sendFirmwareCmd(LASER_TEC_ON);
	sendFirmwareCmd(LASER_OUTPUT_ENABLE);
	sendFirmwareCmd(RX_OFFSET_VOLTAGE_REMOVE);
	sendFirmwareCmd(LASER_OUTPUT_ON);
	sendFirmwareCmd(DATA_ACQUISITION_ON);

	return true;
}

void spectrolab::SpectroScan3D::stop() {
	sendFirmwareCmd(LASER_OUTPUT_OFF);
	sendFirmwareCmd(LASER_TEC_OFF);
	sendFirmwareCmd(LASER_POWER_SUPPLY_OFF);
	sendFirmwareCmd(HIGH_VOLTAGE_OFF);
	sendFirmwareCmd(RESET);
}

void spectrolab::SpectroScan3D::writeFirmware(FirmwareWriteCommands cmd,
		uint8_t data) {
	uint8_t buff[10];
	buff[0]= cmd;
	buff[1]= data;
	cmd_socket_->send(ba::buffer(buff, 2));
	cmd_socket_->receive(ba::buffer(buff,2));
	assert(buff[0]==0);
	assert(buff[1]==0);
}

void spectrolab::SpectroScan3D::handleImgFrame(
		const boost::system::error_code& err) {

	if (err && (err != boost::asio::error::eof) )
    {
      std::cout << "[SpectroScan3D] Error reading image frame : " << err << "\n";
      return;
    }
	std::cout << "Recieved image frame\n";
}
