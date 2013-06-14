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
#include <pcl/io/spectroscan_3d_io.h>
#include "spectroscan_settings_widget.h"

#include <QWidget>
#include <qsettings.h>



class SpectolabViewer : public QMainWindow{
	Q_OBJECT
public:
	SpectolabViewer() ;
	virtual ~SpectolabViewer();

	public slots:
		void loadScan();
		void loadMovie();
		void setFrameRate();
		void spectroscan3dConnect();
		void spectroscan3dSettings();
		void spectroscan3dSettingsApplied();
	private:
	Ui_MainWindow ui_;
	pcl::visualization::CloudPlayerWidget* cplayer_;
	boost::shared_ptr<pcl::Grabber> grabber_;
	float frame_rate_;

	QSettings settings_;
	pcl::SpectroscanSettings spectroscan_settings_;
	SpectroscanSettingsWidget* settings_widget_;

	protected:
		virtual void closeEvent ( QCloseEvent * event );
};

#endif /* VIEWER_APPLICATION_H_ */
