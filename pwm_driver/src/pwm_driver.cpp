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

#include "pwm_driver/pwm_driver.h"

PwmDriver::PwmDriver( bosch_hardware_interface* hw, uint32_t frequency, uint8_t pin ): sensor_driver( hw )
{
  _frequency = frequency;
  _pin = pin;
}

PwmDriver::~PwmDriver()
{
}

uint8_t PwmDriver::getDeviceAddress()
{
  // the answer to all questions
  return 42;
}

bool PwmDriver::initialize()
{  
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("PwmDriver::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  return true;
}

bool PwmDriver::set( float value )
{
  int *flags = NULL; // not needed for PWM
  uint8_t num_bytes = 4;  // only set to make it look nice, not really needed
  uint8_t duty_cycle_chopped[4];
  uint32_t duty_cycle;
  
  if( value < 0.0 || value > 1.0 )
  {
    ROS_ERROR("PWM duty cycle must be between 0 and 1");
    return false;
  }
  
  // convert float to uint32
  double convertion_helper = value;
  duty_cycle = (uint32_t)(convertion_helper * 0xFFFFFFFF);
  // write expects an uint_8 array, chopping uint32 to four uint8_t MSB first
  uint32_t temp;
  temp = (duty_cycle & (0xFF << 24)) >> 24;
  duty_cycle_chopped[0] = (uint8_t)temp;
  temp = (duty_cycle & (0xFF << 16)) >> 16;
  duty_cycle_chopped[1] = (uint8_t)temp;
  temp = (duty_cycle & (0xFF << 8)) >> 8;
  duty_cycle_chopped[2] = (uint8_t)temp;
  temp = (duty_cycle & (0xFF << 0));
  duty_cycle_chopped[3] = (uint8_t)temp;
  
  //std::cout << "duty_cycle: " << duty_cycle << "  chopped: " << duty_cycle_chopped[0] + 0 << " " << duty_cycle_chopped[1] + 0 << " " << duty_cycle_chopped[2] + 0 << " " << duty_cycle_chopped[3] + 0 << " " << std::endl;
  
  if( hardware_->write( this->getDeviceAddress(), PWM, _frequency, flags, _pin, duty_cycle_chopped, num_bytes ) < 0 )
  {
    ROS_ERROR("PwmDriver::setPWM(): could not write PWM to serial device.");
    return false;
  } 
  return true;
}
