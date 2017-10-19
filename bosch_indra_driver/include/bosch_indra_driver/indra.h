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

#ifndef BOSCH_INDRA_DRIVER_INDRA_H
#define BOSCH_INDRA_DRIVER_INDRA_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"
#include "bosch_indra_driver/measurement_report.h"
#include "bosch_indra_driver/measurement_report_config.h"

using std::vector;
using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;
using std::cout;
using std::endl;
using std::string;
using std::stringstream;

namespace bosch_indra_driver {

typedef enum
{
  PARAM   = 0,
  TRANS   = 1,
  OP      = 2,
} INDRA_OP_MODE;

typedef enum
{
  COMMAND_VALUE_ACCEPTANCE,
  OPERATING_MODE_SETTING,
  GOING_TO_ZERO,
  ABSOLUTE_RELATIVE,
  IMMEDIATE_BLOCK_CHANGE,
  CLEAR_ERROR,
  POSITIONING_JOGGING,
  COMMAND_OPERATING_MODE,
  IPOSYNC,
  DRIVE_HALT,
  DRIVE_ENABLE,
  DRIVE_ON,
} CONTROL_FIELD;

typedef enum
{
  OPERATING_MODE_ACKNOWLEDGMENT,
  IN_REFERENCE,
  IN_STANDSTILL,
  COMMAND_VALUE_REACHED,
  COMMAND_CHANGE_BIT,
  OPERATING_MODE_ERROR,
  STATUS_OF_COMMAND_VALUE_PROCESSING,
  ACTUAL_OPERATING_MODE,
  COMMAND_VALUE_ACKNOWLEDGMENT,
  CLASS_3_DIAGNOSTICS_MESSAGE,
  CLASS_2_DIAGNOSTICS_MESSAGE,
  CLASS_1_DIAGNOSTICS_DRIVE_ERROR,
  READY_FOR_OPERATION,
} STATUS_FIELD;

class WORD
{
public:
  uint16_t word;
  uint16_t masks_[16];
  uint16_t shifts_[16];
  string   desc_[16];
  string   change_word;

  WORD():word(0)
  {}

  uint16_t get_field(uint16_t mask, uint16_t shift)
  {
    return (uint16_t) ((word & mask) >> shift);
  }

  void set_field(uint16_t mask, uint16_t shift, uint16_t value)
  {
    uint16_t temp;
    temp = value << shift;
    temp &= mask;
    word &= ~mask;
    word |= temp;
  }

  uint16_t get_by_offset(uint8_t offset)
  {
    return ((word & masks_[offset]) >> shifts_[offset]);
  }

  void set_by_offset(uint8_t offset, uint16_t value)
  {
    word &= ~(masks_[offset]);
    word |= value << shifts_[offset];
  }

  string get_desc_by_value(string desc, uint16_t value) {
    string temp;
    string start;
    uint16_t delim_tmp;

    delim_tmp = desc.find(';');
    start = desc.substr(0, delim_tmp);
    temp = desc.substr(delim_tmp+1);
    for (uint8_t i=0; i<value; i++) {
      temp = temp.substr(temp.find(';')+1);
    }
    temp = temp.substr(0,temp.find(';')); // Trim off the rest
    stringstream sstm;
    sstm << start;
    sstm << " ";
    sstm << change_word;
    sstm << " ";
    sstm << value;
    sstm << " (";
    sstm << temp;
    sstm << ").";
    return sstm.str();
  }
};

class CONTROL_WORD : public WORD
{
public:

  CONTROL_WORD()
  {
    masks_[0] = 1;
    masks_[1] = 2;
    masks_[2] = 4;
    masks_[3] = 8;
    masks_[4] = 16;
    masks_[5] = 32;
    masks_[6] = 192;
    masks_[7] = 768;
    masks_[8] = 4096;
    masks_[9] = 8192;
    masks_[10] = 16384;
    masks_[11] = 32768;

    shifts_[0] = 0;
    shifts_[1] = 1;
    shifts_[2] = 2;
    shifts_[3] = 3;
    shifts_[4] = 4;
    shifts_[5] = 5;
    shifts_[6] = 6;
    shifts_[7] = 8;
    shifts_[8] = 12;
    shifts_[9] = 13;
    shifts_[10] = 14;
    shifts_[11] = 15;

    desc_[0] = "Command value acceptance;toggled;toggled;";
    desc_[1] = "Operating mode setting;parameter mode;operating mode;";
    desc_[2] = "Going to zero;complete command;start homing;";
    desc_[3] = "Positioning command value;absolute;relative;";
    desc_[4] = "Immediate block change;accepted after previous complete;accepted upon toggling command value acceptance;";
    desc_[5] = "Clear error;end clear;start clear;";
    desc_[6] = "Positioning;positioning active;infinite travel in + direction;infinite travel in negative direction;";
    desc_[7] = "Command operating mode;primary mode;secondary mode 1;secondary mode 2;secondary mode 3;";
    desc_[8] = "IPYSYNC;toggled;toggled;";
    desc_[9] = "Drive halt;drive halt;drive start;";
    desc_[10] = "Drive enable;disabled;enabled;";
    desc_[11] = "Drive on/off;off;on;";

    change_word = "set to:";
  }

  void set(CONTROL_FIELD field, uint16_t value)
  {
    set_field((uint16_t)masks_[field],
              (uint16_t)shifts_[field],
              value);
  }

  uint16_t get(CONTROL_FIELD field)
  {
    return get_field((uint16_t)masks_[field],
                     (uint16_t)shifts_[field]);
  }
};

class STATUS_WORD : public WORD
{
public:

  STATUS_WORD()
  {
    masks_[0] = 3;
    masks_[1] = 4;
    masks_[2] = 8;
    masks_[3] = 16;
    masks_[4] = 32;
    masks_[5] = 64;
    masks_[6] = 128;
    masks_[7] = 768;
    masks_[8] = 1024;
    masks_[9] = 2048;
    masks_[10] = 4096;
    masks_[11] = 8192;
    masks_[12] = 49152;

    shifts_[0] = 0;
    shifts_[1] = 2;
    shifts_[2] = 3;
    shifts_[3] = 4;
    shifts_[4] = 5;
    shifts_[5] = 6;
    shifts_[6] = 7;
    shifts_[7] = 8;
    shifts_[8] = 10;
    shifts_[9] = 11;
    shifts_[10] = 12;
    shifts_[11] = 13;
    shifts_[12] = 14;

    desc_[0] = "Operating mode acknowledgement;phase 2 (parameter mode);phase 3;phase 4 (operating mode);";
    desc_[1] = "Encoder in reference;relative;homed;";
    desc_[2] = "In standstill;false;true;";
    desc_[3] = "Command value reached;false;true;";
    desc_[4] = "Command change bit;not changed;changed;";
    desc_[5] = "Operating mode error;no error in transition command;error in transition command;";
    desc_[6] = "Status of command value processing;drive follows command value input;drive does not follow command value input;";
    desc_[7] = "Actual operating mode;primary mode of operation;secondary mode 1;secondary mode 2;secondary mode 3;";
    desc_[8] = "Command value acknowledgement;toggled;toggled;";
    desc_[9] = "Class 3 diagnostics message;not present;present;";
    desc_[10] = "Class 2 diagnostics message;not present;present;";
    desc_[11] = "Class 1 diagnostics drive error;not present;present;";
    desc_[12] = "Ready for operation;not ready for power on;ready for power on;control and power sections ready for operation;in operation;with torque;";

    change_word = "is now:";
  }

  void set(STATUS_FIELD field, uint16_t value)
  {
    throw std::logic_error("ERROR: Trying to set the status word - it is read only.");
  }

  uint16_t get(STATUS_FIELD field)
  {
    return get_field((uint16_t)masks_[field],
                     (uint16_t)shifts_[field]);
  }
};

/**
 * Main interface for the INDRA Laser Scanner. Produces methods to access the
 * laser scanner from a high level, including setting parameters and getting
 * single scans.
 */
class INDRA : public Session
{
public:
  CONTROL_WORD ctrl_wrd, prv_ctrl_wrd;
  STATUS_WORD  stat_wrd, prv_stat_wrd;
  int32_t position;
  int32_t velocity;
  int32_t target_position;
  int32_t target_velocity;
  uint8_t debug;
  uint8_t internal_flag;

  /**
   * Construct a new INDRA instance.
   * @param socket Socket instance to use for communication with the lidar
   */
  INDRA(shared_ptr<Socket> socket, shared_ptr<Socket> io_socket)
    : Session(socket, io_socket), connection_num_(-1), mrc_sequence_num_(1),
    lst_status_word_(0), debug(0), ready_for_cfg_cmd_(true), target_velocity(0),
    target_position(0), internal_flag(0), ready_for_ctrl_cmd_(false),
    first_control_command_(true)
  {}

  void print_status_word_changes();

  void print_control_word_changes();

  void print_status_word();

  void print_control_word();

  void print_status_field(uint8_t index);

  void print_control_field(uint8_t index);

  void sendMeasurmentReportConfigUDP();

  uint8_t set_control_field(CONTROL_FIELD field, uint16_t value, bool respect_lock);

  uint8_t enter_operating_mode();

  uint8_t enter_parameter_mode();

  MeasurementReport receiveMeasurementReportUDP();

  void sync();

  void toggle_command_bits();

  void establishConnection();

  uint8_t in_operating_mode();

  uint8_t in_parameter_mode();

  uint8_t set_absolute_positioning();

  uint8_t set_relative_positioning();

  uint8_t ready_for_command();

  uint8_t drive_off();

  uint8_t drive_on();

  uint8_t home_drive();

  uint8_t halt_on();

  uint8_t halt_off();

  uint8_t has_error();

  uint8_t clear_error();

  uint8_t goto_position(int32_t position, int32_t velocity);

  void align_words();

  void toggleCommandBit();

  bool previous_config_change_acknowledged();

  bool pervious_control_command_acknowledged();


private:

  // data for sending to controller
  int connection_num_;
  MeasurementReportConfig mrc_;
  EIP_UDINT mrc_sequence_num_;
  // internal persistant control word
  uint16_t lst_status_word_; // most recent status word
  uint16_t prv_status_word_; // two words ago
  bool ready_for_cfg_cmd_;
  bool ready_for_ctrl_cmd_;
  bool first_control_command_;
};

} // namespace bosch_indra_driver

#endif  // OMRON_INDRA_DRIVER_INDRA_H
