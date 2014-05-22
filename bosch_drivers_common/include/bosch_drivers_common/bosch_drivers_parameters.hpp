/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *********************************************************************/

//\Author Joshua Vasquez and Philip Roan, Robert Bosch LLC

#ifndef BOSCH_DRIVERS_PARAMETERS_H_
#define BOSCH_DRIVERS_PARAMETERS_H_

#include "bosch_drivers_common.hpp"

namespace bosch_drivers_common
{ 
  /**
   * \brief An abstract class for sensor communication properties to be used by hardware interfaces.
   *
   * If a device requires additional communication properties, it should inherit from this class.
   */
  class bosch_driver_parameters   
  {
  public:
    /**
     * \brief The address or ID of the device.
     *
     * For internally located devices such as GPIO or PWM drivers, this is the pin number on the hardware interface.
     *
     */
    uint8_t device_address;

    /**
     * \brief The communication protocol which the sensor uses to transmit data.
     *
     * Some sensors support multiple protocols. See \link bma180 \endlink.
     */
    interface_protocol protocol;
    
    /**
     * \brief The frequency at which data is being sent on the particular protocol.
     */
    unsigned int frequency;
 
    /**
     * \brief An byte containing bit-order, mode, and the spi chip-select pin.
     *
     * Packing this information into an 8-bit set of flags minimizes the number
     * of transmissions between the computer and the hardware interface.
     */
    uint8_t flags;   
  

    bosch_driver_parameters():
      device_address( 0 ),
      protocol( RS232 ),
      frequency( 0 ),
      flags( 0x00 )
    {
    }
    
    ~bosch_driver_parameters() {};

  };
}
#endif //BOSCH_DRIVERS_PARAMETERS_H_
