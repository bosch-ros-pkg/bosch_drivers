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

#include <bosch_drivers_common.hpp>

namespace bosch_drivers_common
{ 
  /**
   * \brief An abstract class for sensor and hardware interface parameters
   *
   * A specific parameters class for a particular sensor will inherit this class.
   */
  class Parameters   
  {
  public:
    Parameters() {};
    virtual ~Parameters() {};

    /**
     * \brief The protocol which the sensor is configured to transmit data.
     *
     * Some sensors support multiple protocols. See \link bma180 \endlink.
     */
    interface_protocol protocol_;
    
    /**
     * \brief The frequency at which data is being sent on the particular protocol.
     */
    int frequency_;

    /**
     * \brief the pin on the hardware interface which the sensor has connected it's chip-select line.
     *
     * This parameter is only relevant for sensors that support the SPI protocol.
     */
    int pin_;              
 
    /**
     * \brief An integer containing bit-order, mode, and the spi chip-select pin.
     *
     * Packing this information into an 8-bit set of flags minimizes the number
     * of transmissions between the computer and the hardware interface.
     */
    int flags_;   
  
    /**
     * \brief Sets the frequency of the sensor data transmissions between the hardware interface and the sensor.
     */           
    virtual bool setFrequency( int frequency ) = 0;

    /**
     * \brief Retrieve the frequency at which the sensor data transmissions take place.
     */
    virtual int getFrequency() = 0;
 
    /**
     * \brief Select the protocol that both the hardware interface and sensor use to communicate.
     *
     * \p interface_protocol is an enumerated datatype from bosch_drivers_common.
     */
    virtual bool setProtocol( interface_protocol protocol_name ) = 0;
 
    /**
     * \brief Retrieve the communication protocol.
     */
    virtual interface_protocol getProtocol() = 0;

    /**
     * \brief Retreive the flags for communication between hardware interface and sensor.
     */
    virtual int* getFlags() = 0; // SPI only
 
    /**
     * \brief Alert software driver to which pin the sensor's chip-select pin is connected to.
     *
     * Relevant to SPI mode only.
     */
    virtual bool setPin( uint8_t pin ) = 0; // SPI, GPIO only. In read/write methods, the pin is the input to the device_address.
 
    /**
     * \brief Retrieve the hardware pin that the sensor's chip-select pin is connected to.
     *
     * Relevant to SPI mode only.
     */
    virtual int getPin() = 0; // SPI, GPIO only
  };
}
#endif //BOSCH_DRIVERS_PARAMETERS_H_
