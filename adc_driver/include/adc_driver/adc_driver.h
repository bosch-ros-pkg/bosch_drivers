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

//\Author Kai Franke and Philip Roan, Robert Bosch LLC

#ifndef ADC_DRIVER_H_
#define ADC_DRIVER_H_

// ROS headers for debugging output
#include <ros/console.h>
#include <bosch_drivers_common/bosch_drivers_common.hpp>
#include <bosch_drivers_common/bosch_drivers_sensor_driver_internal.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

/**
 * \brief Driver to use an ADC on a supported serial device.
 *
 * This class lets the user access the ADC pins of any supported hardware
 */
class AdcDriver: public sensor_driver_internal
{

public:
  /** Constructor:
   * \brief sets up one pin as an ADC pin with the default reference voltage
   * \param hw Connected hardware interface
   * \param adc_pin Pin to use on the connected hardware
   * \note the ADC reference voltage will be set to the default value of the connected serial device (usually the supply voltage)
   */
  AdcDriver( bosch_hardware_interface* hw, uint8_t adc_pin );
 
  // Destructor:
  ~AdcDriver();

  // Public Driver Methods:
  bool setDeviceAddress( uint8_t new_pin );
  bool setFrequency(unsigned int frequency);

  bosch_driver_parameters getParameters();
  bool setParameters( bosch_driver_parameters parameters );
  
  /**
   * \brief Reads the analog voltage from the connected hardware device
   * \return The read voltage in micro volts [µV]
   */
  uint32_t getVoltage();
  
  /**
   * \brief Sets the reference voltage on the hardware for all analog pins
   * \param voltage The voltage to set the reference to in mV. Check the hardware interface implementation for supported voltages. Setting this parameter to 0 will set the reference type to external
   * \return true if the reference was applied successfully
   * \todo think about switching to µV?
   */
  bool setReference( uint32_t voltage );
	
  /**
   * \brief Initializes the driver and the connected hardware
   * 
   * \return a boolean indicating success
   */
  bool initialize();
  
};

#endif // ADC_DRIVER_H_
