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

#include "encoder_driver/encoder_driver.h"

bool EncoderDriver::encoders_[MAX_ENCODERS];

EncoderDriver::EncoderDriver( bosch_hardware_interface* hw, uint8_t encoder1_pin, uint8_t encoder2_pin ):
  sensor_driver_internal( hw ),
  _encoder1_pin( encoder1_pin ),
  _encoder2_pin(encoder2_pin ),
  _last_position( 0 ),
  _overflow( 0 ),
  invert_( 1 )  
{  
  if( getNextID( &(communication_properties_->device_address) ) )
  {
    std::vector<uint8_t> data(3);

    data[0] = CREATE ;
    data[1] = _encoder1_pin;
    data[2] = _encoder2_pin;  // indicates that we want to create the object on the hardware device
    if( _encoder1_pin == _encoder2_pin )
    {
      ROS_ERROR("EncoderDriver::EncoderDriver(): Encoder pins must be different");
      EncoderDriver::encoders_[communication_properties_->device_address] = false;
    }
    else
    {
      if( hardware_->write( *communication_properties_, ENCODER, data ) < 0 )
      {
        ROS_ERROR("EncoderDriver::~EncoderDriver(): could not create object on hardware device");
	EncoderDriver::encoders_[communication_properties_->device_address] = false;
      }
    }
  }
  else
  {
    ROS_ERROR("Maximum number of Encoder objects(16) exceeded, will not create any more");
  }
}

EncoderDriver::EncoderDriver( bosch_hardware_interface* hw, uint8_t encoder_id ):
  sensor_driver_internal( hw ),
  _encoder1_pin( 255 ),
  _encoder2_pin( 255 ),
  _last_position( 0 ),
  _overflow( 0 ),
  invert_( 1 )  
{ 
  // Determine encoder ID 
  if( communication_properties_->device_address >= MAX_ENCODERS )
  {
    ROS_ERROR("Select an encoder_id [0..15] which was already created on the serial device");
  }
}

// destroys the object created on the hardware device
EncoderDriver::~EncoderDriver()
{
  // check if instance on serial device needs to be destroyed
  if( _encoder1_pin != _encoder2_pin )  // indicates that object was created on serial device
  {
    std::vector<uint8_t> data( 1, DESTROY ); // indicates that we want to destroy the object on the hardware device
    
    if( hardware_->write( *communication_properties_, ENCODER, data ) < 0 )
    {
      ROS_ERROR("EncoderDriver::~EncoderDriver(): could not destroy object on hardware device");
    }
    else
    {
      EncoderDriver::encoders_[communication_properties_->device_address] = false;
    }
  }

  delete communication_properties_;
}

uint8_t EncoderDriver::getDeviceAddress()
{
  return communication_properties_->device_address;
}

bool EncoderDriver::initialize()
{ 
  if( _encoder1_pin == _encoder2_pin )
  {
    ROS_WARN("EncoderDriver::initialize(): This encoder has already been initialized");
    return true;
  }
  
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("PwmDriver::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  // reset encoder position to 0
  if( ! EncoderDriver::setPosition( 0 ) )
    return false;
  
  return true;
}

uint8_t EncoderDriver::getEncoderID( )
{
  return communication_properties_->device_address;
}

int64_t EncoderDriver::getPosition()
{
  std::vector<uint8_t> data(4,0);
  int32_t position; // return value
  
  if( hardware_->read( *communication_properties_, ENCODER, data ) < 0 )
  {
    ROS_ERROR("EncoderDriver::getPosition(): could not read input");
    return false;
  } 
  // convert read data to int32_t
  uint32_t temp[4];
  temp[0] = data[0];
  temp[1] = data[1];
  temp[2] = data[2];
  temp[3] = data[3];
  position  = temp[0] << 24;
  position |= temp[1] << 16;
  position |= temp[2] << 8;
  position |= temp[3] << 0;
  
  if( _last_position > (1 << 30) )  // was last position greater than one billion?
  {
    if( position < 0 ) // assume there was an overflow
    {
      ++_overflow;
    }
  }
  else if( _last_position < (-1) * (1 << 30) )  // was last position smaller than negative one billion?
  {
    if( position > 0 )  // assume there was an underflow
    {
      --_overflow;
    }
  }
  _last_position = position;
  
  return ((_overflow * 4294967296) + position) * invert_ ;
}

bool EncoderDriver::setPosition( int32_t position )
{
  position *= invert_;  // invert if encoder values are inverted, too
  _overflow = 0;  // reset overflow

  std::vector<uint8_t> data(7,0); // will contain split up position
  
  data[0] = SET_POSITION;
  data[1] = _encoder1_pin;
  data[2] = _encoder2_pin;
  
  // split position into 4 bytes for transmission
  uint32_t temp;
  temp = ((position & 0xFF000000) >> 24);
  data[3] = (uint8_t)temp;
  temp = ((position & 0x00FF0000) >> 16);
  data[4] = (uint8_t)temp;
  temp = ((position & 0x0000FF00) >> 8);
  data[5] = (uint8_t)temp;
  temp = ((position & 0x000000FF) >> 0);
  data[6] = (uint8_t)temp;
  
  if( hardware_->write( *communication_properties_, ENCODER, data ) < 0 )
  {
    ROS_ERROR("EncoderDriver::setPosition(): could not write position");
    return false;
  } 
  return true;
}

bool EncoderDriver::zero()
{
  return setPosition( 0 );
}

void EncoderDriver::invertOutput( )
{
  invert_ *= -1;
}

bool EncoderDriver::getNextID( uint8_t* next )
{
  for( int n = 0; n < MAX_ENCODERS; n++ )
  {
    if( EncoderDriver::encoders_[n] == false )
    {
      EncoderDriver::encoders_[n] = true;
      *next = n;
      return true;
    }
  }
  return false;
}

bool EncoderDriver::setDeviceAddress( uint8_t address)
{
  communication_properties_->device_address = address;
  return true;
}
 
bosch_drivers_communication_properties EncoderDriver::getParameters()
{
  return *communication_properties_;
}

bool EncoderDriver::setParameters( bosch_drivers_communication_properties properties)
{
  *communication_properties_ = properties;
}
