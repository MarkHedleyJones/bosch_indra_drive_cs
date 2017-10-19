/**
Software License Agreement (BSD)

Copyright (c) 2017, Mark Hedley Jones
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
*/


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>

#include "bosch_indra_driver/indra.h"
#include "odva_ethernetip/serialization/serializable_buffer.h"
#include "odva_ethernetip/cpf_packet.h"
#include "odva_ethernetip/cpf_item.h"
#include "odva_ethernetip/sequenced_address_item.h"
#include "odva_ethernetip/sequenced_data_item.h"

using std::cout;
using std::endl;
using boost::shared_ptr;
using boost::make_shared;
using boost::asio::buffer;
using eip::Session;
using eip::serialization::SerializableBuffer;
using eip::RRDataResponse;
using eip::CPFItem;
using eip::CPFPacket;
using eip::SequencedAddressItem;
using eip::SequencedDataItem;


#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')


namespace bosch_indra_driver {


void INDRA::sendMeasurmentReportConfigUDP()
{
  // Detect changes to the control word and toggle the command change flag.
  // if (prv_ctrl_wrd.word != ctrl_wrd.word) toggleCommandBit();

  // TODO from upstream author: check that connection is valid
  CPFPacket pkt;
  shared_ptr<SequencedAddressItem> address =
    make_shared<SequencedAddressItem>(
      getConnection(connection_num_).o_to_t_connection_id, mrc_sequence_num_++);
  shared_ptr<MeasurementReportConfig> data = make_shared<MeasurementReportConfig>();
  *data = mrc_;
  data->sequence_num = mrc_sequence_num_;
  data->control_word = ctrl_wrd.word;
  data->position_command_val = target_position;
  data->position_velocity = target_velocity;
  pkt.getItems().push_back(CPFItem(0x8002, address));
  pkt.getItems().push_back(CPFItem(0x00B1, data));
  sendIOPacket(pkt);
  prv_ctrl_wrd.word = ctrl_wrd.word;
}

void INDRA::print_control_field(uint8_t index)
{
  cout << "CONTROL: ";
  cout << ctrl_wrd.get_desc_by_value(ctrl_wrd.desc_[index], ctrl_wrd.get_by_offset(index));
  cout << endl;
}

void INDRA::print_status_field(uint8_t index)
{
  cout << "STATUS:  ";
  cout << stat_wrd.get_desc_by_value(stat_wrd.desc_[index], stat_wrd.get_by_offset(index));
  cout << endl;
}

void INDRA::print_control_word()
{
  for (uint8_t i=0; i < 12; i++) print_control_field(i);
}

void INDRA::print_status_word()
{
  for (uint8_t i=0; i < 13; i++) print_status_field(i);
}

void INDRA::print_control_word_changes()
{
  for (uint8_t i=0; i < 12; i++)
  {
    if ((ctrl_wrd.word & ctrl_wrd.masks_[i]) != (prv_ctrl_wrd.word & prv_ctrl_wrd.masks_[i]))
    {
      print_control_field(i);
    }
  }
}

void INDRA::print_status_word_changes()
{
  for (uint8_t i=0; i < 13; i++)
  {
    if ((stat_wrd.word & stat_wrd.masks_[i]) != (prv_stat_wrd.word & prv_stat_wrd.masks_[i]))
    {
      print_status_field(i);
    }
  }
}

void INDRA::toggle_command_bits()
{
  if (ctrl_wrd.get(IPOSYNC) == 0)
  {
    ctrl_wrd.set(IPOSYNC, 1);
  }
  else ctrl_wrd.set(IPOSYNC, 0);
}

uint8_t INDRA::set_control_field(CONTROL_FIELD field, uint16_t value, bool respect_lock)
{
  static bool echo = true;

  if (ctrl_wrd.get(field) != value)
  {
    if (respect_lock && previous_config_change_acknowledged() == 0)
    {
      if (echo) {
        cout << "Waiting for target to become ready for new commands." << endl;
        echo = false;
      }
      return 0;
    }
    else
    {
      ctrl_wrd.set(field, value);
      if (respect_lock)
      {
        if (debug) {
          cout << " -- DEVICE LOCKED FOR COMMANDS --" << endl;
          cout << "    * the locking command has a ctrl_wrd offset of " << field << endl;
        }
        ready_for_cfg_cmd_ = false;
      }
      toggle_command_bits();
      return 0;
    }
  }
  else
  {
    echo = true;
    return 1;
  }
}

uint8_t INDRA::goto_position(int32_t position, int32_t velocity)
{
  first_control_command_ = false;
  target_position = position;
  target_velocity = velocity;
  ready_for_ctrl_cmd_ = false;
  if (ctrl_wrd.get(COMMAND_VALUE_ACCEPTANCE) == 0)
  {
    ctrl_wrd.set(COMMAND_VALUE_ACCEPTANCE, 1);
  }
  else ctrl_wrd.set(COMMAND_VALUE_ACCEPTANCE, 0);
  if (debug)
  {
    cout << "Command value sent to target" << endl;
    cout << "COMMAND: Position: " << position << endl;
    cout << "COMMAND: Velocity: " << velocity << endl;
    cout << " -- DEVICE NOT READY FOR CONTROL COMMAND --" << endl;
  }
}

uint8_t INDRA::set_relative_positioning()
{
  ctrl_wrd.set(ABSOLUTE_RELATIVE, 1);
  return true;
}

uint8_t INDRA::ready_for_command()
{
  static bool echo = true;

  if (first_control_command_)
  {
    if (stat_wrd.get(COMMAND_VALUE_REACHED)) {
      // May not be the first command that the drive has received since bootup
      first_control_command_ = false;
    }

    if (stat_wrd.get(READY_FOR_OPERATION) == 3) {
      echo = true;
      return true;
    }
    else
    {
      if (echo) cout << "Drive not ready for operation yet, command not sent";
      return false;
    }
  }
  else if (pervious_control_command_acknowledged()) {
    if (ctrl_wrd.get(IMMEDIATE_BLOCK_CHANGE) == 0)
    {
      // positioning command value (S-0-0282) is only accepted
      // after the last active target position was reached
      if (stat_wrd.get(COMMAND_VALUE_REACHED))
      {
        echo = true;
        return true;
      }
      else
      {
        if (debug && echo)
        {
          cout << "Previous command value not reached, waiting..." << endl;
          echo = false;
        }
        return false;
      }
    }
    else
    {
      // positioning command value (S-0-0282) is immediately
      // accepted upon toggling of command value acceptance
      echo = true;
      return true;
    }
  }
  else
  {
    if (debug && echo)
    {
      cout << "Previous control command not acknowledged. Waiting..." << endl;
      echo = false;
    }
    return false;
  }
}

uint8_t INDRA::set_absolute_positioning()
{
  ctrl_wrd.set(ABSOLUTE_RELATIVE, 0);
  return true;
}

uint8_t INDRA::halt_off()
{
  if (ctrl_wrd.get(DRIVE_HALT) != 1)
  {
    ctrl_wrd.set(DRIVE_HALT, 1);
    return 0;
  }
  else return 1;
}

uint8_t INDRA::halt_on()
{
  if (ctrl_wrd.get(DRIVE_HALT) != 0)
  {
    ctrl_wrd.set(DRIVE_HALT, 0);
    return 0;
  }
  else return 1;
}

uint8_t INDRA::drive_on()
{
  static bool echo = true;
  switch (stat_wrd.get(READY_FOR_OPERATION))
  {
    case 0:
    // Not ready for power on
    if (echo)
    {
      cout << "NOTICE: Waiting for drive to become ready for power up" << endl;
      echo = false;
    }
    return 0;
    break;

    case 1:
    set_control_field(DRIVE_ON, 1, false); // Dont lock the drive with this cmd
    return 0;
    //Ready for power on
    break;

    case 2:
    set_control_field(DRIVE_ON, 1, false); // Dont lock the drive with this cmd
    cout << "NOTICE: Drive ready for power up" << endl;
    echo = true;
    return 1;
    //Control and power sections ready for operation and torque-free
    break;

    case 3:
    cout << "NOTICE: Drive ready for power up" << endl;
    echo = true;
    return 1;
    //In operation, with torque
  }
}

uint8_t INDRA::drive_off()
{
  switch (stat_wrd.get(READY_FOR_OPERATION))
  {
    case 0:
    // Not ready for power on
    return 1;

    case 1:
    //Ready for power on
    return 1;

    case 2:
    //Control and power sections ready for operation and torque-free
    return 0;

    case 3:
    //In operation, with torque
    set_control_field(DRIVE_ON, 0, true);
    return 0;

    default:
      throw std::logic_error("ERROR: Drive in unexpected state of readiness!");
      return -1;
  }
}

uint8_t INDRA::home_drive()
{
  if (ctrl_wrd.get(GOING_TO_ZERO) == 0)
  {
    set_control_field(GOING_TO_ZERO, 1, true);
    return 0;
  }
  else if (stat_wrd.get(IN_REFERENCE) == 1)
  {
    return 1;
  }
  else return 0;
}

uint8_t INDRA::in_operating_mode()
{
  return (stat_wrd.get(OPERATING_MODE_ACKNOWLEDGMENT) == 2);
}

uint8_t INDRA::in_parameter_mode()
{
  return (stat_wrd.get(OPERATING_MODE_ACKNOWLEDGMENT) == 0);
}

uint8_t INDRA::has_error()
{
  static bool echo = true;
  if (stat_wrd.get(CLASS_1_DIAGNOSTICS_DRIVE_ERROR) != 0)
  {
    if (echo) {
      cout << "Target contains errors" << endl;
      echo = false;
    }
    return 1;
  }
  else
  {
    if (echo)
    {
      cout << "Target is error free" << endl;
      echo = true;
    }
    return 0;
  }
}

uint8_t INDRA::clear_error()
{
  static bool echo = true;

  if (has_error()) {
    set_control_field(CLEAR_ERROR, 1, true);
    if (echo) {
      cout << "Clearing error on target" << endl;
      echo = false;
    }
    return 0;
  }
  else {
    set_control_field(CLEAR_ERROR, 0, true);
    cout << "Error cleared" << endl;
    echo = true;
    return 1;
  }
}

uint8_t INDRA::enter_operating_mode()
{
  if (ctrl_wrd.get(OPERATING_MODE_SETTING) != 1)
  {
    set_control_field(OPERATING_MODE_SETTING, 1, true);
    return 0;
  }
  else
  {
    switch (stat_wrd.get(OPERATING_MODE_ACKNOWLEDGMENT))
    {
      case 0:
        // Is in parameter mode
        set_control_field(OPERATING_MODE_SETTING, 1, true);
        return 0;
        break;

      case 1:
        // In in a transitional mode; wait
        return 0;
        break;

      case 2:
        // Is in operating mode; no action required
        return 1;
        break;

      default:
        throw std::logic_error("ERROR: Drive in unexpected operating mode!");
        return -1;
    }
  }
}

uint8_t INDRA::enter_parameter_mode()
{
  if (ctrl_wrd.get(OPERATING_MODE_SETTING) != 0)
  {
    set_control_field(OPERATING_MODE_SETTING, 0, true);
    return 0;
  }
  else
  {
    switch (stat_wrd.get(OPERATING_MODE_ACKNOWLEDGMENT))
    {
      case 0:
        // Is in parameter mode; no action required
        return 1;
        break;

      case 1:
        // In in a transitional mode; wait
        return 0;
        break;

      case 2:
        // Is in operating mode
        set_control_field(OPERATING_MODE_SETTING, 0, true);
        return 0;
        break;

      default:
        throw std::logic_error("ERROR: Drive in unexpected operating mode!");
        return -1;
    }
  }
}

bool INDRA::previous_config_change_acknowledged()
{
  return ready_for_cfg_cmd_;
}

bool INDRA::pervious_control_command_acknowledged()
{
  return ready_for_ctrl_cmd_;
}


void INDRA::sync()
{
  sendMeasurmentReportConfigUDP();

  MeasurementReport out = receiveMeasurementReportUDP();
  if (debug) print_status_word_changes();

  if (stat_wrd.get(COMMAND_CHANGE_BIT) != prv_stat_wrd.get(COMMAND_CHANGE_BIT)) {
    ready_for_cfg_cmd_ = true;
    if (debug) cout << " -- DEVICE READY FOR CONFIG COMMANDS --" << endl;
  }
  if (stat_wrd.get(COMMAND_VALUE_ACKNOWLEDGMENT) != prv_stat_wrd.get(COMMAND_VALUE_ACKNOWLEDGMENT)) {
    ready_for_ctrl_cmd_ = true;
    if (debug) cout << " -- DEVICE READY FOR CONTROL COMMAND --" << endl;
  }
  if (debug) print_control_word_changes();
  velocity = out.velocity_feedback_value;
  position = out.active_position_feedback_value;
}


void INDRA::toggleCommandBit()
{
  if (ctrl_wrd.get(COMMAND_VALUE_ACCEPTANCE)) {
    ctrl_wrd.set(COMMAND_VALUE_ACCEPTANCE, 0);
  }
  else
  {
    ctrl_wrd.set(COMMAND_VALUE_ACCEPTANCE, 1);
  }
}

MeasurementReport INDRA::receiveMeasurementReportUDP()
{
  CPFPacket pkt = receiveIOPacket();
  if (pkt.getItemCount() != 2)
  {
    throw std::logic_error("IO Packet received with wrong number of items");
  }
  if (pkt.getItems()[1].getItemType() != 0x00B1)
  {
    throw std::logic_error("IO Packet received with wrong data type");
  }
  SequencedDataItem<MeasurementReport> data;
  pkt.getItems()[1].getDataAs(data);

  prv_stat_wrd.word = stat_wrd.word;
  stat_wrd.word = data.status_word;

  return data;
}

// Sets the control word bits to the current state of the status words
void INDRA::align_words()
{
  switch (stat_wrd.get(OPERATING_MODE_ACKNOWLEDGMENT))
  {
    case 0:
      ctrl_wrd.set(OPERATING_MODE_SETTING, 0);
      break;

    case 1:
      cout << "WARNING: Couldn't determine operating mode of target" << endl;
      break;

    case 2:
      ctrl_wrd.set(OPERATING_MODE_SETTING, 1);
      break;
  }
}

void INDRA::establishConnection()
{
  EIP_CONNECTION_INFO_T o_to_t, t_to_o;
  // Originator to Target (PC to Drive)
  o_to_t.assembly_id = 101;     // Default set by Indra config tool
  o_to_t.buffer_size = 20;      // 20 Bytes sent to drive
  o_to_t.rpi =         2000;    // 2000 us (2 ms) between messages sent

  // Target to Originator (Drive to PC)
  t_to_o.assembly_id = 102;     // Default set by Indra config tool
  t_to_o.buffer_size = 16;      // 16 Bytes sent in response
  t_to_o.rpi =         2000;    // 2000 us (2 ms) between messages received

  connection_num_ = createConnection(o_to_t, t_to_o);
  sendMeasurmentReportConfigUDP();
  receiveMeasurementReportUDP();
  align_words();
  if (debug) {
    cout << " -- Current CONTROL word -- " << endl;
    print_control_word();
    cout << " -- Current STATUS word -- " << endl;
    print_status_word();
    cout << " -- // -- " << endl;
  }
}

} // namespace indra
