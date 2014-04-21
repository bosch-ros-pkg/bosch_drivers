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

#ifndef BOSCH_DRIVERS_SENSOR_DRIVER_H_
#define BOSCH_DRIVERS_SENSOR_DRIVER_H_


#include <bosch_drivers_common.hpp>
#include <bosch_drivers_parameters.hpp>
#include <bosch_drivers_hardware_interface.hpp>

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
  class sensor_driver    
  {

  public:
    /**
     * Constructor: ties the sensor to its hardware interface
     */
    sensor_driver( bosch_hardware_interface* hw )
    {
      hardware_ = hw;
    }
  
    // Destructor
    virtual ~sensor_driver() {}; 

    /**
     * \brief Retrieve the address of the sensor
     *
     * \note  For SPI transactions, the \p device_address is the chip select. If
     * the device address is \p NULL_DEVICE, then the hardware interface should
     * read from the bus without changing any chip select lines.
     * \note  For GPIO transactions, the \p device_address is the pin or port.
     */
    virtual uint8_t getDeviceAddress() = 0; // depends on the parameters 
 
  protected:
    bosch_hardware_interface* hardware_;
  };
}
#endif //BOSCH_DRIVERS_SENSOR_DRIVER_H_
