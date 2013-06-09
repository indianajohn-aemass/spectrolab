/*
 * viewer_application.h
 *
 *  Created on: Jun 5, 2013
 *      Author: asher
 */

#ifndef VIEWER_APPLICATION_H_
#define VIEWER_APPLICATION_H_

#include "ui_viewer_main.h"
#include <pcl/visualization/cloud_player_widget.h>
#include <QWidget>

class SpectolabViewer : public QMainWindow{
	Q_OBJECT
public:
	SpectolabViewer() ;
	virtual ~SpectolabViewer();

	public slots:
		void loadScan();
		void loadMovie();
		void setFrameRate();
	private:
	Ui_MainWindow ui_;
	pcl::visualization::CloudPlayerWidget* cplayer_;
	boost::shared_ptr<pcl::Grabber> grabber_;
	float frame_rate_;
};

#endif /* VIEWER_APPLICATION_H_ */
