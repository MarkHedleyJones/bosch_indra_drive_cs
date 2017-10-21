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

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"
#include "bosch_indra_driver/indra.h"

#define BOOT 0
#define RUNNING 1
#define ERROR 2

using std::cout;
using std::endl;
using boost::shared_ptr;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;
using namespace bosch_indra_driver;


void print_change(string message, uint8_t &print_flag, string marker)
{
  if (print_flag) {
    cout << marker << " " << message << endl;
    print_flag = false;
  }
}

void print_substate_change(string message, uint8_t &print_flag)
{
  print_change(message, print_flag, "  *");
}

void print_state_change(string message, uint8_t &print_flag)
{
  print_change(message, print_flag, " **");
}

bool initialise_drive(INDRA &indra)
{
  static uint8_t substate = 0;
  static uint8_t prev_substate = 255;
  static uint8_t substate_change_flag = true;

  switch (substate)
  {
    case 0:
      print_substate_change("Checking target in parameter mode.",\
                            substate_change_flag);
      if (indra.in_operating_mode()) substate += 1;
      else if (indra.in_parameter_mode()) substate += 2;
      else cout << "Target in transition state, waiting..." << endl;
      break;

    case 1:
      print_substate_change("Putting target into parameter mode.",\
                            substate_change_flag);
      if (indra.enter_parameter_mode()) substate += 1;
      break;

    case 2:
      print_substate_change("Checking target for errors...",\
                            substate_change_flag);
      if (indra.has_error()) substate += 1;
      else substate += 2;
      break;

    case 3:
      print_substate_change("Target has error. Clearing...",\
                            substate_change_flag);
      if (indra.clear_error()) substate += 1;
      break;

    case 4:
      print_substate_change("Putting drive into set_relative_positioning mode",\
                            substate_change_flag);
      indra.set_relative_positioning();
      substate += 1;
      break;

    case 5:
      print_substate_change("Putting drive into operational mode mode",\
                            substate_change_flag);
      if (indra.enter_operating_mode()) substate += 1;
      break;

    case 6:
      print_substate_change("Enabling drive",\
                            substate_change_flag);
      if (indra.drive_on()) substate += 1;
      break;

    case 7:
      print_substate_change("Removing dive halt flag",\
                            substate_change_flag);
      if (indra.halt_off()) substate += 1;
      break;

    case 8:
      print_substate_change("Bootup complete, finishing up.",\
                            substate_change_flag);
      substate = 0;
      return true;
      break;

    default:
      cout << "ERROR: You should never get here!!!!!!!!!!" << endl;
      return -1;
  }

  if (prev_substate != substate) {
    substate_change_flag = true;
    prev_substate = substate;
  }

  return false;
}

int main(int argc, char *argv[])
{
  uint32_t loop_counter = 0;
  uint8_t state = 0;
  uint8_t prev_state = 0;
  uint8_t state_change_flag = true;
  int8_t direction = 1;

  ros::init(argc, argv, "indra");
  ros::NodeHandle nh;

  // get config from params
  string host;
  ros::param::param<std::string>("~host", host, "192.168.178.76");

  boost::asio::io_service io_service;
  shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
  shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, 2222));
  INDRA indra(socket, io_socket);

  try
  {
    indra.open(host);
  }
  catch (std::runtime_error ex)
  {
    ROS_FATAL_STREAM("Exception caught opening session: " << ex.what());
    return -1;
  }

  indra.debug = 0;

  try
  {
    indra.establishConnection();
  }
  catch (std::logic_error ex)
  {
    ROS_FATAL_STREAM("Could not start UDP IO: " << ex.what());
    return -1;
  }

  while (ros::ok())
  {
    loop_counter++;
    if (loop_counter % 1000 == 0) cout << loop_counter << endl;
    try
    {
      // Give drive a few syncs first to establish current state
      if (loop_counter > 100)
      {
        switch (state) {
          case BOOT:
            print_state_change("Entered BOOT state.", state_change_flag);
            if (initialise_drive(indra)) state = RUNNING;
            break;

          case RUNNING:
            print_state_change("Entered RUNNING State", state_change_flag);
            if (indra.ready_for_command()) {
              if (direction == 1) direction = -1;
              else direction = 1;
              indra.goto_position(direction * 10000000, 10000000);
            }
            break;

          case ERROR:
            print_state_change("Entered ERROR State", state_change_flag);
            break;
        }

        if (prev_state != state) {
          state_change_flag = true;
          prev_state = state;
        }

      }

      // Synchronise messages with the controller
      indra.sync();

      // Detect error in the drive and automatically change state
      if (state == RUNNING && \
          indra.stat_wrd.get(CLASS_1_DIAGNOSTICS_DRIVE_ERROR) != 0)
      {
        state = ERROR;
      }

    }
    catch (std::runtime_error ex)
    {
      ROS_ERROR_STREAM("Exception caught communicating with drive: " << ex.what());
    }
    catch (std::logic_error ex)
    {
      ROS_ERROR_STREAM("Problem parsing data: " << ex.what());
    }

    ros::spinOnce();
  }

  indra.closeConnection(0);
  indra.close();
  return 0;
}
