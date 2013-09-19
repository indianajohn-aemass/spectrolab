#include "cmd_interface_widget.h"
#include <sstream>
#include <stdio.h>
#include <iomanip>

using namespace spectrolab;

CMDAction::CMDAction(SpectroScan3D::FirmwareCommands fcmd, const QString &_text)
    : cmd_binary(fcmd),
      cmd_type(SEND),
      text(_text) {

}

CMDAction::CMDAction(SpectroScan3D::FirmwareReadCommands fcmd,
                     const QString &_text)
    : cmd_binary(fcmd),
      cmd_type(READ),
      text(_text) {
}

CMDAction::CMDAction(SpectroScan3D::FirmwareWriteCommands fcmd,
                     const QString &_text)
    : cmd_binary(fcmd),
      cmd_type(WRITE),
      text(_text) {

}

void CMDInterfaceWidget::addSpectrolabAction(const CMDAction& action) {
  ui_.combo_box_cmds->addItem(action.text);
  this->actions_.push_back(action);
}

CMDInterfaceWidget::CMDInterfaceWidget(
    const boost::shared_ptr<pcl::Spectroscan3DGrabber> &grabber,
    QWidget *parent)
    : grabber_(grabber),
      QWidget(parent) {
  ui_.setupUi(this);

  addSpectrolabAction(CMDAction(SpectroScan3D::RESET, "Reset 0x10"));
  addSpectrolabAction(CMDAction(SpectroScan3D::HIGH_VOLTAGE_ON, "High Voltage On 0x3A"));
  addSpectrolabAction(CMDAction(SpectroScan3D::MIRROR_SCAN_ON, "Mirror Scan On 0x21"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_HIGH_POWER_ON, "Laser high power 0x1A"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_TEC_ON, "Laser Tec On 0x15"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_OUTPUT_ENABLE, "Laser Output Enable 0x17"));
  addSpectrolabAction(CMDAction(SpectroScan3D::DATA_ACQUISITION_ON, "Data Acquisition On 0x11"));
  addSpectrolabAction(CMDAction(SpectroScan3D::RX_OFFSET_VOLTAGE_REMOVE, "RX Offset Voltage Remove 0x3F"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_OUTPUT_ON, "Laser Output On 0x12"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_OUTPUT_OFF, "Laser Output Off 0x18"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_TEC_OFF, "Laser TEC Off 0x16"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_POWER_SUPPLY_OFF, "Laser Power Supply Off 0x13"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_LOW_POWER_ON, "Laser Low power on 0x19"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LED1_TOGGLE, "LED 1 Toggle 0x1F"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_STATUS_DISPLAY, "Laser Status Display 0x2D"));
  addSpectrolabAction(CMDAction(SpectroScan3D::SYSTEM_STATUS_DISPLAY, "System Status Display 0x2E"));
  addSpectrolabAction(CMDAction(SpectroScan3D::INTERPOLATOR_ON, "Interpolator On On 0x31"));
  addSpectrolabAction(CMDAction(SpectroScan3D::INTERPOLATOR_OFF, "Interpolator Off 0x39"));
  addSpectrolabAction(CMDAction(SpectroScan3D::ADC_CALIBRATE, "ADC Calibrate 0x36"));
  addSpectrolabAction(CMDAction(SpectroScan3D::REFERENCE_PULSE_ON, "Reference Pulse On 0x37"));
  addSpectrolabAction(CMDAction(SpectroScan3D::REFERENCE_PULSE_OFF, "Reference Pulse Off 0x38"));
  addSpectrolabAction(CMDAction(SpectroScan3D::HIGH_VOLTAGE_OFF, "High Voltage Off 0x3B"));
  addSpectrolabAction(CMDAction(SpectroScan3D::MIRROR_SCAN_OFF, "Mirror Scan Off 0x22")); 

  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_STATUS_READ, "Read Laser Status 0x25"));
  addSpectrolabAction(CMDAction(SpectroScan3D::SYSTEM_STATUS_READ, "Read System Status 0x26"));
  addSpectrolabAction(CMDAction(SpectroScan3D::SYSTEM_ID_READ, "Read System ID 0x27"));
  addSpectrolabAction(CMDAction(SpectroScan3D::IN_FIFO_DELAY_READ, "Read Fifo Delay 0x30"));

  addSpectrolabAction(CMDAction(SpectroScan3D::ODD_SYNC_DELAY_WRITE, "Write odd sync delay 0x23"));
  addSpectrolabAction(CMDAction(SpectroScan3D::EVEN_SYNC_DELAY_WRITE, "Write even sync delay 0x2F"));
  addSpectrolabAction(CMDAction(SpectroScan3D::HIGH_GAIN_AMPLITUDE_THRESHOLD_WRITE, "Write high gain amplitude threshold 0x24"));
  addSpectrolabAction(CMDAction(SpectroScan3D::HIGH_GAIN_AMPLITUDE_SATURATE_WRITE, "Write high gain amplitude saturate 0x34"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LOW_GAIN_AMPLITUDE_THRESHOLD_WRITE, "Write low gain amplitude threshold 0x28"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LOW_GAIN_AMPLITUDE_SATURATE_WRITE, "Write low gain amplitude saturate 0x35"));
  addSpectrolabAction(CMDAction(SpectroScan3D::IN_FIFO_DELAY_WRITE, "Write FIFO delay 0x2A"));
  addSpectrolabAction(CMDAction(SpectroScan3D::TARGET_SELECT_WRITE, "Write Target Select 0x33"));

  connect(ui_.button_send, SIGNAL(clicked()), this, SLOT(sendCMD() ));
  connect(ui_.combo_box_cmds, SIGNAL(currentIndexChanged(int)), this, SLOT(cmdSelected(int)));
  connect(ui_.line_edit_cmd, SIGNAL(returnPressed()), this, SLOT(sendCMD() ));
}

void CMDInterfaceWidget::sendCMD() {
  int action_idx = ui_.combo_box_cmds->currentIndex();

  try {
    QString display_string;
    if (actions_[action_idx].cmd_type == CMDAction::WRITE) {
      uint32_t val;
      QByteArray str = this->ui_.line_edit_cmd->text().toLocal8Bit();
      if ((str.data()[0] == '0') && (str.data()[1] == 'x')) {
        sscanf(str.data(), "%x", &val);
      } else {
        sscanf(str.data(), "%d", &val);
      }
      grabber_->getDriver().writeFirmware(
          (SpectroScan3D::FirmwareWriteCommands) actions_[action_idx].cmd_binary,
          val);
      display_string += actions_[action_idx].text;
      char cmd_text[20];
      sprintf(cmd_text, " (0x%x 0x%x )\n",
              (int) actions_[action_idx].cmd_binary, val);
      display_string += cmd_text;
    } else {
      if (actions_[action_idx].cmd_type == CMDAction::READ) {
        uint8_t val;
        grabber_->getDriver().readFirmware(
            (SpectroScan3D::FirmwareReadCommands) actions_[action_idx]
                .cmd_binary,
            val);
      } else {
        grabber_->getDriver().sendFirmwareCmd(
            (SpectroScan3D::FirmwareCommands) actions_[action_idx].cmd_binary);
      }
      display_string += actions_[action_idx].text;
      char cmd_text[20];
      sprintf(cmd_text, " ( 0x%x )\n", (int) actions_[action_idx].cmd_binary);
      display_string += cmd_text;
    }
    std::stringstream ss;
    std::vector<uint8_t> response;
    response = grabber_->getDriver().getLastResponse();
    for (int i = 0; i < response.size(); i++) {
      ss << "0x" << std::hex << (int) response[i] << " ";
    }
    ss << "\n";
    display_string += ss.str().c_str();
    ui_.response_browser->appendPlainText(display_string);

  } catch (std::exception& e) {
    QString display_string = actions_[action_idx].text + "\n";
    display_string += e.what();
    display_string += "\n";
    ui_.response_browser->appendPlainText(display_string);
  }
}

void CMDInterfaceWidget::cmdSelected(int idx) {
  if (idx >= 0) {
    if (actions_[idx].cmd_type == CMDAction::WRITE) {
      ui_.line_edit_cmd->setEnabled(true);
    } else {
      ui_.line_edit_cmd->setEnabled(false);
    }
  }
}
