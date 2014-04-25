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

EncoderDriver::EncoderDriver( bosch_hardware_interface* hw, uint8_t encoder1_pin, uint8_t encoder2_pin ): sensor_driver( hw )
{
  // variable to automaticly define encoder object id starting with 0
  static uint8_t encoder_id = 0;
  
  _encoder1_pin = encoder1_pin;
  _encoder2_pin = encoder2_pin;
  _encoder_id = encoder_id;
  _last_position = 0;
  _overflow = 0;
  invert_ = 1;
  
  // Determine encoder ID 
  if (encoder_id < 16)
  {
    int frequency = 0; // does not apply for Encoder
    int flags[3] = { CREATE, _encoder1_pin, _encoder2_pin };  // indicates that we want to create the object on the hardware device
    flags[0] |= encoder_id << 4; // writes the object id to the upper 4 bits. encoder_id will tell the hardware device which object to create
    uint8_t reg = 0; // does not apply for Encoder
    uint8_t data[0]; // not needed for constructor
    uint8_t num_bytes = 0; // will not be read
    if( _encoder1_pin == _encoder2_pin )
    {
      ROS_ERROR("EncoderDriver::EncoderDriver(): Encoder pins must be different");
    }
    else
    {
      if( hardware_->write( getDeviceAddress(), ENCODER, frequency, flags, reg, data, num_bytes ) < 0 )
      {
        ROS_ERROR("EncoderDriver::~EncoderDriver(): could not create object on hardware device");
      }
    }
    ++encoder_id; // increase number ob created objects
  }
  else
  {
    ROS_ERROR("Maximum number of Encoder objects(16) exceeded, will not create any more");
  }
}

EncoderDriver::EncoderDriver( bosch_hardware_interface* hw, uint8_t encoder_id ): sensor_driver( hw )
{
  // set encoder pins to same value to indicate that there was no object created on the serial device
  _encoder1_pin = _encoder2_pin = 255;
  // assign this instance to the desired instance on the serial device
  _encoder_id = encoder_id; 
  _last_position = 0;
  _overflow = 0;
  invert_ = 1;
  
  // Determine encoder ID 
  if (encoder_id >= 16)
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
    int frequency = 0; // does not apply for Encoder
    int flags[1] = { DESTROY }; // indicates that we want to destroy the object on the hardware device
    flags[0] |= _encoder_id << 4; // writes the object id to the upper 4 bits. encoder_id will tell the hardware device which object to destroy
    uint8_t reg = 0; // does not apply for Encoder
    uint8_t data[0]; // not needed for destructor
    uint8_t num_bytes = 0; // will not be read
    
    if( hardware_->write( getDeviceAddress(), ENCODER, frequency, flags, reg, data, num_bytes ) < 0 )
    {
      ROS_ERROR("EncoderDriver::~EncoderDriver(): could not destroy object on hardware device");
    }
  }
}

uint8_t EncoderDriver::getDeviceAddress()
{
  // the answer to all questions + 3
  return 45;
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
  if(! EncoderDriver::setPosition(0) )
    return false;
  
  return true;
}

uint8_t EncoderDriver::getEncoderID( )
{
  return _encoder_id;
}

int64_t EncoderDriver::getPosition()
{
  int frequency = 0; // does not apply for Encoder
  int flags[1];
  flags[0] = _encoder_id << 4; // writes the object id to the upper 4 bits. encoder_id will tell the hardware device which encoder object to read from
  uint8_t reg = 0; // does not apply for Encoder
  uint8_t data[4]; // will store the data after successful read call
  uint8_t num_bytes = 4; // will not be read because it is always 4 bytes
  int32_t position; // return value
  
  if( hardware_->read( getDeviceAddress(), ENCODER, frequency, flags, reg, data, num_bytes ) < 0 )
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
  int frequency = 0; // does not apply for Encoder
  int flags[3] = { SET_POSITION, _encoder1_pin, _encoder2_pin };
  flags[0] |= _encoder_id << 4; // writes the object id to the upper 4 bits. encoder_id will tell the hardware device which encoder object to write to
  uint8_t reg = 0; // does not apply for Encoder
  uint8_t data[4]; // will contain split up position
  uint8_t num_bytes = 4; // will not be read because it is always 4 bytes
  
  // split position into 4 bytes for transmission
  uint32_t temp;
  temp = ((position & 0xFF000000) >> 24);
  data[0] = (uint8_t)temp;
  temp = ((position & 0x00FF0000) >> 16);
  data[1] = (uint8_t)temp;
  temp = ((position & 0x0000FF00) >> 8);
  data[2] = (uint8_t)temp;
  temp = ((position & 0x000000FF) >> 0);
  data[3] = (uint8_t)temp;
  
  if( hardware_->write( getDeviceAddress(), ENCODER, frequency, flags, reg, data, num_bytes ) < 0 )
  {
    ROS_ERROR("EncoderDriver::setPosition(): could not write position");
    return false;
  } 
  return true;
}

void EncoderDriver::invertOutput( )
{
  invert_ *= -1;
}
