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

//\Author Philip Roan, Robert Bosch LLC

#ifndef BOSCH_DRIVERS_SENSOR_DRIVER_BUILT_IN_H_
#define BOSCH_DRIVERS_SENSOR_DRIVER_BUILT_IN_H_


#include "bosch_drivers_common.hpp"
#include "bosch_drivers_communication_properties.hpp"
#include "bosch_drivers_hardware_interface.hpp"
#include "bosch_drivers_sensor_driver.hpp"

namespace bosch_drivers_common
{
  /**
   * \brief An abstract class for all Bosch sensor drivers.
   *
   * This class provides the generic format so that sensors can communicate over
   * different physical hardware devices. All sensor drivers must inherit this
   * class, ensuring that all sensors are tied to a hardware interface when
   * instantiated.
   */
  class sensor_driver_built_in: public sensor_driver
  {
  public:
    /**
     * Constructor: ties the sensor to its hardware interface
     */
    sensor_driver_built_in( bosch_hardware_interface* hw ):
      sensor_driver( hw, BUILT_IN_DEVICE )
    {
    }

    /**
     * Constructor: ties the sensor to its hardware interface
     */
    sensor_driver_built_in( bosch_hardware_interface* hw, bosch_drivers_communication_properties* properties ):
      sensor_driver( hw, BUILT_IN_DEVICE, properties )
    {
      communication_properties_->protocol = BUILT_IN;
    }

    
    // Destructor
    virtual ~sensor_driver_built_in()
    {
      delete communication_properties_;
    }
    
    /**
     * \brief Retrieve the address of the sensor
     *
     * \note  For SPI transactions, the \p device_address is the chip select. If
     * the device address is \p NULL_DEVICE, then the hardware interface should
     * read from the bus without changing any chip select lines.
     * \note  For GPIO transactions, the \p device_address is the pin or port.
     */
    virtual uint8_t getDeviceAddress()
    {
      return communication_properties_->device_address;
    }
    
    virtual bool setDeviceAddress( uint8_t address ) = 0;

    /**
     * \brief Sets the frequency of the sensor data transmissions between the hardware interface and the sensor.
     */           
    virtual bool setFrequency( unsigned int frequency )
    {
      return false; // internal device does not have a communication frequency
    }
    
    /**
     * \brief Retrieve the frequency at which the sensor data transmissions take place.
     */
    virtual unsigned int getFrequency()
    {
      return 0; // internal device does not have a communication frequency
    }
    
    /**
     * \brief Select the protocol that both the hardware interface and sensor use to communicate.
     *
     * \p interface_protocol is an enumerated datatype from bosch_drivers_common.
     */
    virtual bool setProtocol( interface_protocol protocol_name )
    {
      return false; // Built-in device does not have a communication protocol.
    }
    
    /**
     * \brief Retrieve the communication protocol.
     */
    virtual interface_protocol getProtocol()
    {
      return communication_properties_->protocol;
    }
    
    /**
     * \brief Retreive the flags for communication between hardware interface and sensor.
     */
    virtual uint8_t getFlags()
    {
      return 0x00; // Built-in device does not have communication flags.
    }
    
    virtual bosch_drivers_communication_properties getParameters()
    {
      return *communication_properties_;
    }

    virtual bool setParameters( bosch_drivers_communication_properties properties ) = 0;
  };
  
}
#endif //BOSCH_DRIVERS_SENSOR_DRIVER_BUILT_IN_H_
