#ifndef CMD_INTERFACE_WIDGET_H
#define CMD_INTERFACE_WIDGET_H

#include <QWidget>
#include <Q3Action>
#include <pcl/io/spectroscan_3d_io.h>
#include <ui_cmd_interface.h>

class CMDAction {
    public:
        CMDAction( spectrolab::SpectroScan3D::FirmwareCommands fcmd,
                             const QString& text);
        CMDAction( spectrolab::SpectroScan3D::FirmwareReadCommands fcmd,
                             const QString& text);
        CMDAction( spectrolab::SpectroScan3D::FirmwareWriteCommands fcmd,
                             const QString& _text);
        enum CMDType { SEND, READ, WRITE};
        CMDType cmd_type; // just send (0), read(1), write (2)
        uint8_t cmd_binary;
        QString text;
};

class CMDInterfaceWidget : public QWidget
{
    Q_OBJECT
public:
    CMDInterfaceWidget( const boost::shared_ptr<pcl::Spectroscan3DGrabber>&grabber,
                             QWidget *parent = 0);

signals:
    
private slots:
    void cmdSelected(int idx);
    void sendCMD();

private:
    Ui_CMDInterface ui_;
    boost::shared_ptr<pcl::Spectroscan3DGrabber> grabber_;




    void addSpectrolabAction(const CMDAction& action);
    std::vector<CMDAction> actions_;
};


#endif // CMD_INTERFACE_WIDGET_H
