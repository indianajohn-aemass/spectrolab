/*
 * viewer_application.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: Adam Stambler
 */

#include "viewer_application.h"
#include <pcl/io/spectroscan_3d_io.h>
#include <pcl/io/file_grabber.h>
#include <boost/filesystem.hpp>
#include <QFileDialog>

#include <pcl/io/movie_grabber.h>

SpectolabViewer::SpectolabViewer() {
	ui_.setupUi(this);
	cplayer_= new pcl::visualization::CloudPlayerWidget(ui_.centralwidget);
	QObject::connect(ui_.actionLoad_movie, SIGNAL(triggered()), this , SLOT(loadMovie()) );
	QObject::connect(ui_.actionLoad_scan, SIGNAL(triggered()), this , SLOT(loadScan()) );
}

SpectolabViewer::~SpectolabViewer() {
	delete cplayer_;
}
#include <qlabel.h>

void SpectolabViewer::loadMovie() {
	std::cout << "Loading Movie!\n";
	QString fileName = QFileDialog::getOpenFileName(this, tr("Select a frame of the movie"),
	                                                 "",
	                                                 tr("Files (*.pcd *.bin)"));
	boost::filesystem::path frame_path = fileName.toAscii().data();

	grabber_.reset(new pcl::MovieGrabber( frame_path.parent_path(), frame_path.extension().string() ) );
	this->cplayer_->setGrabber(grabber_);
}

void SpectolabViewer::loadScan() {
	std::cout << "Loading a scan!\n";
	QString fileName = QFileDialog::getOpenFileName(this, tr("Select PCD or Spectroscan image frame"),
	                                                 "",
	                                                 tr("Files (*.pcd *.bin)"));


}


