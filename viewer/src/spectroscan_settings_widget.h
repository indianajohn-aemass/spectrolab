/*
 * spectroscan_settings_widget.h
 *
 *  Created on: Jun 12, 2013
 *      Author: asher
 */

#ifndef SPECTROSCAN_SETTINGS_WIDGET_H_
#define SPECTROSCAN_SETTINGS_WIDGET_H_

#include <QWidget>
#include "ui_spectroscan_settings.h"

#include <pcl/io/spectroscan_3d_io.h>

class SpectroscanSettingsWidget : public QWidget{
	Q_OBJECT
public:
	SpectroscanSettingsWidget(pcl::SpectroscanSettings* settings);

signals:
	void settingsApplied();

private slots:
		void applySettings();
		void save();
		void load();

protected:
	Ui_SpectroscanSettings ui_;
	pcl::SpectroscanSettings* settings_;
};


#endif /* SPECTROSCAN_SETTINGS_WIDGET_H_ */
