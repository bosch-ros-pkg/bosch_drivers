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

#ifndef PWM_DRIVER_H_
#define PWM_DRIVER_H_

#include <ros/console.h> // ROS headers for debugging output

#include <bosch_drivers_common/bosch_drivers_common.hpp>
#include <bosch_drivers_common/bosch_drivers_sensor_driver_built_in.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

/**
 * \brief Driver to output a PWM on a supported serial device.
 *
 * This class lets the user access the PWM pins of any supported hardware
 * to apply a PWM to supported hardware passing the duty cycle in as a value
 * between 0 (constant low) and 1 (constant high)
 */
class PwmDriver: public sensor_driver_built_in
{
  
public:
  /** 
   * \brief Constructor:
   * \param  frequency PWM frequency
   * \param  pin pin number on the hardware device to apply the PWM to
   */
  PwmDriver( bosch_hardware_interface* hw, unsigned int frequency, uint8_t pin, unsigned int resolution_in_bits );
  
  // Destructor:
  ~PwmDriver();
  
  bool setDeviceAddress( uint8_t pin );

  bool setParameters( bosch_drivers_communication_properties properties);

  /**
   * \brief Sends the duty cycle \a value to a supported serial device
   * \param  value duty cycle as fraction [0..1] with 0 being constant LOW and 1 being constant HIGH
   * \return true if write was successful or false if not
   */
  bool setDutyCycle( double value );
  double getDutyCycle();

  unsigned int getModulationFrequency( );
  bool setModulationFrequency( unsigned int frequency );
  
  bool setResolution( unsigned int bits );
  unsigned int getResolution();

  /**
   * \brief Initializes the driver and the connected hardware
   * 
   * \return a boolean indicating success
   */
  bool initialize();

private:
  unsigned int modulation_frequency_;

  typedef unsigned long long pwm_resolution_t;
  unsigned int resolution_bits_;
  size_t resolution_bytes_;

  double duty_cycle_;
  std::vector<uint8_t> duty_cycle_bytes_;  

  bool convertDutyCycle();
  bool sendUpdate();

};

#endif // PWM_DRIVER_H_

