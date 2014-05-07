/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Robert Bosch LLC.
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

//\Author Kai Franke, Robert Bosch LLC

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

// ROS headers for debugging output
#include <ros/console.h>
#include <bosch_drivers_common/bosch_drivers_common.hpp>
#include <bosch_drivers_common/bosch_drivers_sensor_driver.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

/**
 * \brief Driver to use a GPIO on a supported serial device.
 *
 * This class lets the user access the GPIO pins of any supported hardware
 * It is possible to configure the pin as Output, floating Input, Input with Pullup or Input with Pulldown
 * Depending on the hardware used some of these options might not be available
 * Once the GPIO is configured it can be set to HIGH or LOW if output
 * or can be read if configured as an input
 */
class GpioDriver: public sensor_driver
{

public:
  /**
   * \brief Constructor
   * \param hw hardware interface to use
   * \param pin GPIO pin number on the hardware device
   */
  GpioDriver( bosch_hardware_interface* hw, uint8_t pin );

  // Destructor:
  ~GpioDriver();


  uint8_t getDeviceAddress( void ); 
  bool setDeviceAddress( uint8_t address );

  unsigned int getFrequency();
  bool setFrequency( unsigned int frequency );

  interface_protocol getProtocol();
  bool setProtocol( interface_protocol protocol_name );
  
  uint8_t getFlags();
  bosch_driver_parameters getParameters();

  /**
   * \brief Set a GPIO Pin to HIGH or LOW. It will always configure the pin as an output first.
   * \param value 0 will set the GPIO pin to LOW and 1 will set it to HIGH
   * \return true if GPIO output was successful or false if not
   */
  bool setOutput( bool value );
  
  /**
   * \brief Performs a digital read on the GPIO Pin returning the read value in \a value
   * \param mode configures the input pin to be either floating, add a pullup or a pulldown
   * \return Read value at given pin. 1 if selected Pin is HIGH and 0 if it is LOW
   */
  bool getInput( gpio_input_mode mode );
  
  /**
   * \brief Initializes the driver and the connected hardware
   * 
   * \return a boolean indicating success
   */
  bool initialize();
};

#endif // GPIO_DRIVER_H_

