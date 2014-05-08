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

PwmDriver::PwmDriver( bosch_hardware_interface* hw, unsigned int frequency, uint8_t pin, unsigned int resolution_in_bits ):
  sensor_driver( hw ),
  duty_cycle_( 0.0 )
{
  sensor_parameters_ = new bosch_driver_parameters();
  sensor_parameters_->protocol = PWM;
  sensor_parameters_->frequency = frequency;
  sensor_parameters_->device_address = pin;
  sensor_parameters_->flags = 0x00;

  setResolution( resolution_in_bits );
}

PwmDriver::~PwmDriver()
{
  delete sensor_parameters_;
}

uint8_t PwmDriver::getDeviceAddress()
{
  return sensor_parameters_->device_address;
}

bool PwmDriver::setDeviceAddress( uint8_t new_pin )
{
  sensor_parameters_->device_address = new_pin;

  // do some other stuff?

  return true;
}

unsigned int PwmDriver::getFrequency()
{
  return sensor_parameters_->frequency;
}

bool PwmDriver::setFrequency( unsigned int new_frequency )
{
  sensor_parameters_->frequency = new_frequency;

  return true;
}

bool PwmDriver::setResolution( unsigned int bits )
{
  resolution_bytes_ = ceil( bits / 8 );
  if( resolution_bytes_ > sizeof( pwm_resolution_t ) )
  {
    ROS_ERROR("PwmDriver: The maximum resolution on this architecture is %ld bits. You have requested %ld bits.", 8*sizeof( pwm_resolution_t ), 8*resolution_bytes_ );
    return false;
  }

  duty_cycle_bytes_.resize( resolution_bytes_ );
  resolution_bits_ = bits;

  return true;
}

unsigned int PwmDriver::getResolution()
{
  return resolution_bits_;
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

bool PwmDriver::setDutyCycle( double value )
{ 
  if( value < 0.0 || value > 1.0 )
  {
    ROS_ERROR("PWM duty cycle must be between 0 and 1");
    return false;
  }
  duty_cycle_ = value;

  convertDutyCycle();
 
  if( !sendUpdate() )
  {
    ROS_WARN("PwmDriver: update not sent.");
    return false;
  }
  return true;
}

bool PwmDriver::convertDutyCycle()
{
  // convert float to uint32
  //double convertion_helper = value;
  //uint32_t dc_fixed; 
  //dc_fixed = (uint32_t)(convertion_helper * 0xFFFFFFFF);
  // write expects an uint_8 array, chopping uint32 to four uint8_t MSB first
  //uint32_t temp;
  //temp = (dc_fixed & (0xFF << 24)) >> 24;
  //duty_cycle_bytes_[0] = (uint8_t)temp;
  //temp = (dc_fixed & (0xFF << 16)) >> 16;
  //duty_cycle_bytes_[1] = (uint8_t)temp;
  //temp = (dc_fixed & (0xFF << 8)) >> 8;
  //duty_cycle_bytes_[2] = (uint8_t)temp;
  //temp = (dc_fixed & (0xFF << 0));
  //duty_cycle_bytes_[3] = (uint8_t)temp;

 
  pwm_resolution_t temp = (duty_cycle_ * static_cast<double>( 1<<resolution_bits_) );
  for( unsigned int i = 0; i < duty_cycle_bytes_.size(); i++ )
  {
    duty_cycle_bytes_[i] = static_cast<uint8_t>( temp  >> 8*(resolution_bytes_ - i) );
  }
  
  //std::cout << "duty cycle: " << duty_cycle_ << "  chopped: " << duty_cycle_bytes_[0] + 0 << " " << duty_cycle_bytes_[1] + 0 << " " << duty_cycle_bytes_[2] + 0 << " " << duty_cycle_bytes[3] + 0 << " " << std::endl;
  return true;
}

bool PwmDriver::sendUpdate()
{
  if( hardware_->write( *sensor_parameters_, 0x00, &duty_cycle_bytes_[0], duty_cycle_bytes_.size() ) < 0 )
  {
    ROS_ERROR("PwmDriver::sendUpdate(): could not write PWM to serial device.");
    return false;
  } 
  return true;
}

  
bool PwmDriver::setProtocol( interface_protocol protocol_name )
{
  if( protocol_name != PWM )
    return false;

  sensor_parameters_->protocol = protocol_name;
  return true;
}

interface_protocol PwmDriver::getProtocol()
{
  return sensor_parameters_->protocol;
}
  
uint8_t PwmDriver::getFlags()
{
  return sensor_parameters_->flags;
}
 
bosch_driver_parameters PwmDriver::getParameters()
{
  return *sensor_parameters_;
}

bool PwmDriver::setParameters( bosch_driver_parameters parameters)
{
  *sensor_parameters_ = parameters;
}
