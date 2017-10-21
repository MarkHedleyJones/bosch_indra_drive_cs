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

#ifndef OMRON_INDRA_DRIVER_MEASUREMENT_REPORT_CONFIG_H
#define OMRON_INDRA_DRIVER_MEASUREMENT_REPORT_CONFIG_H

#include <string>

#include "odva_ethernetip/eip_types.h"
#include "odva_ethernetip/serialization/reader.h"
#include "odva_ethernetip/serialization/writer.h"
#include "odva_ethernetip/serialization/serializable.h"

using std::string;
using eip::serialization::Serializable;
using eip::serialization::Reader;
using eip::serialization::Writer;

namespace bosch_indra_driver {

/**
 * Data structure and operators for INDRA specific Measurement Report Header
 * as defined in the INDRA-DM Ethernet/IP Addendum
 */
class MeasurementReportConfig : public Serializable
{
public:
  EIP_UINT sequence_num;           // 2 byte
  EIP_UDINT run_idle;              // 4 byte
  EIP_UINT control_word;           // 2 byte
  EIP_DINT position_command_val;   // 4 byte
  EIP_DINT position_velocity;      // 4 byte

  MeasurementReportConfig() : sequence_num(1), run_idle(3),
    control_word(0), position_command_val(0), position_velocity(0)
  {
  }

  /**
   * From INDRA-DM Ethernet/IP addendum, header is always 56 bytes
   */
  virtual size_t getLength() const
  {
    return 20;
  }

  /**
   * Serialize data into the given buffer
   * @param writer Writer to use for serialization
   * @return the writer again
   * @throw std::length_error if the buffer is too small for the header data
   */
  virtual Writer& serialize(Writer& writer) const
  {
    writer.write(sequence_num);
    writer.write(run_idle);
    writer.write(control_word);
    writer.write(position_command_val);
    writer.write(position_velocity);
    return writer;
  }

  /**
   * Extra length information is not relevant in this context. Same as deserialize(reader)
   */
  virtual Reader& deserialize(Reader& reader, size_t length)
  {
    return deserialize(reader);
  }

  /**
   * Deserialize data from the given reader without length information
   * @param reader Reader to use for deserialization
   * @return the reader again
   * @throw std::length_error if the buffer is overrun while deserializing
   */
  virtual Reader& deserialize(Reader& reader)
  {
    reader.read(sequence_num);
    reader.read(run_idle);
    reader.read(control_word);
    reader.read(position_command_val);
    reader.read(position_velocity);
    return reader;
  }
};

} // namespace bosch_indra_driver

#endif  // OMRON_INDRA_DRIVER_MEASUREMENT_REPORT_CONFIG_H
