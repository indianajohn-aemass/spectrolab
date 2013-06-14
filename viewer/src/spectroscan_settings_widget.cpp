/*
 * spectroscan_settings_widget.cpp
 *
 *  Created on: Jun 12, 2013
 *      Author: asher
 */

#include "spectroscan_settings_widget.h"

#include <qfiledialog.h>

SpectroscanSettingsWidget::SpectroscanSettingsWidget(
		pcl::SpectroscanSettings* settings) {
	settings_ = settings;
	ui_.setupUi(this);
	connect(ui_.button_apply, SIGNAL(clicked()), this, SLOT(applySettings() ));
	connect(ui_.button_save, SIGNAL(clicked()), this, SLOT(save() ));
	connect(ui_.button_load, SIGNAL(clicked()), this, SLOT(load() ));

	ui_.spin_box_max_range->setValue(settings_->max_range);
	ui_.spin_box_min_range->setValue(settings_->min_range);
	ui_.spin_box_y_angle->setValue(settings_->y_angle_delta);
	ui_.spin_box_x_angle->setValue(settings_->x_angle_delta);
	ui_.spin_box_range_offset->setValue(settings_->range_offset);
	ui_.spin_box_x_angle->setValue(settings_->x_angle_delta);
}

void SpectroscanSettingsWidget::applySettings() {
	bool ok;
	double tmp;
	tmp = ui_.spin_box_max_range->text().toDouble(&ok);
	if( ok) settings_->max_range=tmp;
	else ui_.spin_box_max_range->setValue(settings_->max_range);

	tmp = ui_.spin_box_min_range->text().toDouble(&ok);
	if( ok) settings_->min_range=tmp;
	else ui_.spin_box_min_range->setValue(settings_->min_range);

	tmp = ui_.spin_box_range_offset->text().toDouble(&ok);
	if( ok) settings_->range_offset=tmp;
	else ui_.spin_box_range_offset->setValue(settings_->range_offset);

	tmp = ui_.spin_box_x_angle->text().toDouble(&ok);
	if( ok) settings_->x_angle_delta=tmp;
	else ui_.spin_box_x_angle->setValue(settings_->x_angle_delta);

	tmp = ui_.spin_box_y_angle->text().toDouble(&ok);
	if( ok) settings_->y_angle_delta=tmp;
	else ui_.spin_box_y_angle->setValue(settings_->y_angle_delta);

	emit settingsApplied();
}

void SpectroscanSettingsWidget::save() {
	QString fname = QFileDialog::getSaveFileName(this, tr("Save settings to which file?"), "" ,
			tr("Files (*.ini)"));
	if (fname.isEmpty()) return;
	settings_->save(fname.toAscii().data());
}

void SpectroscanSettingsWidget::load() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open a Spectroscan 3D settings file"),
	                                                "",
	                                                 tr("Files (*.ini)"));
	if (fileName.size() ==0) return;
	boost::filesystem::path frame_path = fileName.toAscii().data();
	if (! settings_->load(frame_path.string()) ) {
		return;
	}
	ui_.spin_box_max_range->setValue(settings_->max_range);
	ui_.spin_box_min_range->setValue(settings_->min_range);
	ui_.spin_box_y_angle->setValue(settings_->y_angle_delta);
	ui_.spin_box_x_angle->setValue(settings_->x_angle_delta);
	ui_.spin_box_range_offset->setValue(settings_->range_offset);
}
