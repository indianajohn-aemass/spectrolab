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
#include <QInputDialog>

#include <pcl/io/movie_grabber.h>

SpectolabViewer::SpectolabViewer() : frame_rate_(5){
	ui_.setupUi(this);
	this->ui_.centralwidget->setLayout(new QVBoxLayout);
	cplayer_= new pcl::visualization::CloudPlayerWidget();
	this->ui_.centralwidget->layout()->addWidget(cplayer_);
	cplayer_->addCloudRenderer(new pcl::visualization::CloudRendererRange("intensity"));
	cplayer_->setRenderer(cplayer_->getNumRenderers()-1);

	QObject::connect(ui_.action_movie_load, SIGNAL(triggered()), this , SLOT(loadMovie()) );
	QObject::connect(ui_.action_movie_frame_rate, SIGNAL(triggered()), this , SLOT(setFrameRate()) );
	QObject::connect(ui_.actionLoad_scan, SIGNAL(triggered()), this , SLOT(loadScan()) );
	this->setWindowTitle("Spectrolab Viewer");
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

	if (frame_path.extension()==".pcd"){
		grabber_.reset(new pcl::MovieGrabber( frame_path.parent_path(), frame_path.extension().string() ) );
	}
	else{
		grabber_.reset(new pcl::Spectroscan3DMovieGrabber( frame_path.parent_path() ) );
	}
	dynamic_cast<pcl::MovieGrabber*>( grabber_.get())->setFramesPerSecond(frame_rate_);
	this->cplayer_->setGrabber(grabber_);
}

void SpectolabViewer::loadScan() {
	std::cout << "Loading a scan!\n";
	QString fileName = QFileDialog::getOpenFileName(this, tr("Select PCD or Spectroscan image frame"),
	                                                 "",
	                                                 tr("Files (*.pcd *.bin)"));


}

void SpectolabViewer::setFrameRate(){
	bool ok;
	int val =QInputDialog::getInteger(NULL, "How many frames per second should movies play?", "FPS: ", 4, 1, 30, 1, &ok);
	if (ok) frame_rate_=val;

	pcl::MovieGrabber* grabber = 	dynamic_cast<pcl::MovieGrabber*>( grabber_.get());
	if (grabber!= NULL){
		grabber->setFramesPerSecond(frame_rate_);
	}
}

