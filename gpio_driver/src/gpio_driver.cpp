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

#include "gpio_driver/gpio_driver.h"

GpioDriver::GpioDriver( bosch_hardware_interface* hw, uint8_t pin ): sensor_driver( hw )
{
  _pin = pin;  
}

GpioDriver::~GpioDriver()
{
}

uint8_t GpioDriver::getDeviceAddress()
{
  // the answer to all questions and one more
  return 43;
}

bool GpioDriver::initialize()
{  
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("GpioDriver::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  return true;
}

bool GpioDriver::set( bool value )
{
  int *flags = NULL; //not needed for output
  int frequency = 0; // does not apply for GPIO
  uint8_t num_bytes = 1;
  uint8_t data[1] = {(bool)value};
  if( hardware_->write( this->getDeviceAddress(), GPIO, frequency, flags, _pin, data, num_bytes ) < 0 )
  {
    ROS_ERROR("GpioDriver::setOutput(): could not set output");
    return false;
  } 
  return true;
}

bool GpioDriver::get( gpio_input_mode mode )
{
  int flags[1] = {mode};
  int frequency = 0; // does not apply for GPIO
  uint8_t num_bytes = 1;
  uint8_t data[1];
  
  if( hardware_->read( this->getDeviceAddress(), GPIO, frequency, flags, _pin, data, num_bytes ) < 0 )
  {
    ROS_ERROR("GpioDriver::readInput(): could not read input");
    return false;
  } 
  return (bool)data[0];
}


