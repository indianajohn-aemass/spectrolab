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

#ifndef VIEWER_APPLICATION_H_
#define VIEWER_APPLICATION_H_

#include "ui_viewer_main.h"
#include <pcl/visualization/cloud_player_widget.h>
#include <pcl/io/spectroscan_3d_io.h>
#include "spectroscan_settings_widget.h"
#include "cmd_interface_widget.h"

#include <QWidget>
#include <qsettings.h>

class SpectolabViewer : public QMainWindow
{
  Q_OBJECT
  public:
    SpectolabViewer ();
    virtual ~SpectolabViewer ();

  public slots:
    void loadScan ();
    void loadMovie ();
    void setFrameRate ();
    void spectroscan3dConnect ();
    void spectroscan3dSettings ();
    void spectroscan3dSettingsApplied ();

    void commandInterface();

  private:
    Ui_MainWindow ui_;
    pcl::visualization::CloudPlayerWidget* cplayer_;
    boost::shared_ptr<pcl::Grabber> grabber_;
    float frame_rate_;

    QSettings settings_;
    pcl::SpectroscanSettings spectroscan_settings_;
    SpectroscanSettingsWidget* settings_widget_;

    CMDInterfaceWidget* interface_widget_;

  protected:
    virtual void closeEvent (QCloseEvent * event);
};

#endif /* VIEWER_APPLICATION_H_ */
