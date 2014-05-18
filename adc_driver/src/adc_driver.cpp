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

#include "adc_driver/adc_driver.h"

AdcDriver::AdcDriver( bosch_hardware_interface* hw, uint8_t adc_pin ): sensor_driver( hw )
{
  //sensor_parameters_ = new bosch_driver_parameters();
  sensor_parameters_->protocol = ADCONVERTER;
  sensor_parameters_->device_address = adc_pin;
  sensor_parameters_->frequency = 0;
  sensor_parameters_->flags = 0x00;
}

AdcDriver::~AdcDriver()
{
  //delete sensor_parameters_;
}

bool AdcDriver::setDeviceAddress( uint8_t new_pin )
{
  sensor_parameters_->device_address = new_pin;

  // do some other stuff?

  return true;
}

bool AdcDriver::setFrequency(unsigned int frequency)
{
  return false;
}
 
bool AdcDriver::setProtocol( interface_protocol protocol )
{
  if( protocol != ADCONVERTER )
    return false;
  
  sensor_parameters_->protocol = ADCONVERTER;
  return true;
}

bool AdcDriver::setParameters( bosch_driver_parameters parameters )
{
  *sensor_parameters_ = parameters;

  return true;
}



bool AdcDriver::initialize()
{  
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("AdcDriver::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  return true;
}

uint32_t AdcDriver::read()
{
  uint8_t num_bytes = 4;
  uint8_t data[4];
  
  if( hardware_->read( *sensor_parameters_, 0x00, data, num_bytes ) < 0 )
  {
    ROS_ERROR("AdcDriver::read(): could not read input");
    return 0;
  } 
  // convert read data to float
  uint32_t temp[4];
  uint32_t adc_uV;
  temp[0] = data[0];
  temp[1] = data[1];
  temp[2] = data[2];
  temp[3] = data[3];
  adc_uV  = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + temp[3] ;
  
  /* the following code would allow returning the voltage as a float but is most likely not very portable because of the reinterpret cast
  return *reinterpret_cast<float *>( &adc_uV );
  */
  return adc_uV;
}

bool AdcDriver::setReference( uint32_t voltage )
{
  uint8_t num_bytes = 4;
  uint8_t voltage_chopped[4];
  
  // write expects an uint_8 array, chopping uint32 to four uint8_t MSB first
  uint32_t temp;
  temp = (voltage & (0xFF << 24)) >> 24;
  voltage_chopped[0] = (uint8_t)temp;
  temp = (voltage & (0xFF << 16)) >> 16;
  voltage_chopped[1] = (uint8_t)temp;
  temp = (voltage & (0xFF << 8)) >> 8;
  voltage_chopped[2] = (uint8_t)temp;
  temp = (voltage & (0xFF << 0));
  voltage_chopped[3] = (uint8_t)temp;
  
  if( hardware_->write( *sensor_parameters_, 0x00, voltage_chopped, num_bytes ) < 0 )
  {
    ROS_ERROR("AdcDriver::setReference(): could not write reference voltage");
    return false;
  } 
  return true;
}
