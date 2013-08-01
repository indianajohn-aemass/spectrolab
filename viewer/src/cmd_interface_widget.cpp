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

  addSpectrolabAction(CMDAction(SpectroScan3D::RESET, "Reset"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::MIRROR_SCAN_ON, "Mirror Scan On"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::MIRROR_SCAN_OFF, "Mirror Scan Off"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_HIGH_POWER_ON, "Laser high power on"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LASER_TEC_ON, "Laser Tec On"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_OUTPUT_ENABLE, "Laser Output Enable"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_OUTPUT_ON, "Laser Output On"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_OUTPUT_OFF, "Laser Output Off"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_TEC_OFF, "Laser Tech Off"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_POWER_SUPPLY_OFF,
                "Laser Power Supply Off"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_LOW_POWER_ON, "Laser Low power on"));
  addSpectrolabAction(CMDAction(SpectroScan3D::LED1_TOGGLE, "LED 1 Toggle"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_STATUS_DISPLAY, "Laser Status Display"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::SYSTEM_STATUS_DISPLAY, "System Status Display"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::INTERPOLATOR_ON, "Interpolator On On"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::INTERPOLATOR_OFF, "Interpolator Off"));
  addSpectrolabAction(CMDAction(SpectroScan3D::ADC_CALIBRATE, "ADC Calibrate"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::REFERENCE_PULSE_ON, "Reference Pulse On"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::REFERENCE_PULSE_OFF, "Reference Pulse Off"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::HIGH_VOLTAGE_ON, "High Voltage On"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::HIGH_VOLTAGE_OFF, "High Voltage Off"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::RX_OFFSET_VOLTAGE_REMOVE,
                "RX Offset Voltage Remove"));

  addSpectrolabAction(
      CMDAction(SpectroScan3D::LASER_STATUS_READ, "Read Laser Status"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::SYSTEM_STATUS_READ, "Read System Status"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::SYSTEM_ID_READ, "Read System ID"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::IN_FIFO_DELAY_READ, "Read In Fifo"));

  addSpectrolabAction(
      CMDAction(SpectroScan3D::ODD_SYNC_DELAY_WRITE, "Write odd sync delay"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::EVEN_SYNC_DELAY_WRITE, "Wrote even sync delay"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::HIGH_GAIN_AMPLITUDE_THRESHOLD_WRITE,
                "Write high gain amplitude threshold"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::HIGH_GAIN_AMPLITUDE_SATURATE_WRITE,
                "Write high gain amplitude saturate"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LOW_GAIN_AMPLITUDE_THRESHOLD_WRITE,
                "Write low gain amplitude threshold"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::LOW_GAIN_AMPLITUDE_SATURATE_WRITE,
                "Write low gain amplitude saturate"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::IN_FIFO_DELAY_WRITE, "Write in FIFO delay"));
  addSpectrolabAction(
      CMDAction(SpectroScan3D::TARGET_SELECT_WRITE, "Write Target Select"));

  connect(ui_.button_send, SIGNAL(clicked()), this, SLOT(sendCMD() ));
  connect(ui_.combo_box_cmds, SIGNAL(currentIndexChanged(int)), this,
          SLOT(cmdSelected(int)));
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
