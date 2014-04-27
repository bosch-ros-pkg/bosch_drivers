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

/*********************************************************************
 *
 * Disclaimer
 *
 * This source code is not officially released or supported by Bosch Sensortec.
 * Please contact the listed authors with bugs or other modifications.
 * If you would like the official Bosch Sensortec drivers, please contact:
 * contact@bosch-sensortec.com
 *
 *********************************************************************/

//\Author Joshua Vasquez and Philip Roan, Robert Bosch LLC

#include "bmp085_driver/bmp085_parameters.hpp"


/**********************************************************************/
// Constructor
/**********************************************************************/
BMP085Parameters::BMP085Parameters() 
{
  // set defaults:
  this->setFrequency( 400000 );
  this->setProtocol( I2C );
  this->setSamplingMode( STANDARD );
  flags_ = 0;  // unused for this sensor since it does not support SPI.
}


/**********************************************************************/
// Destructor
/**********************************************************************/
BMP085Parameters::~BMP085Parameters()
{
}


/**********************************************************************/
/**********************************************************************/
bool BMP085Parameters::setSamplingMode( sampling_mode mode )
{
  oss_ = mode;
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMP085Parameters::setProtocol( interface_protocol protocol )
{
  // This function must exist since it is an inherited virtual function.
  if( protocol != I2C )
  {
    ROS_ERROR( "Cannot change protocol from default: I2C.  No other protocols are supported.");
    return false;
  }
  else
    protocol_ = protocol;
  
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMP085Parameters::setFrequency( int frequency )
{
  frequency_ = frequency;
  return true;
}


/**********************************************************************/
/**********************************************************************/
interface_protocol BMP085Parameters::getProtocol()
{
  return protocol_;
}


/**********************************************************************/
/**********************************************************************/
int BMP085Parameters::getFrequency()
{
  return frequency_;
}


/**********************************************************************/
/**********************************************************************/
bool BMP085Parameters::setPin( uint8_t pin)
{
  pin_ = pin;
  return true;
}


/**********************************************************************/
/**********************************************************************/
int BMP085Parameters::getPin()
{
  return pin_;
}


/**********************************************************************/
/**********************************************************************/
int* BMP085Parameters::getFlags()
{
  return &flags_;
}


/**********************************************************************/
/**********************************************************************/
BMP085Parameters::sampling_mode BMP085Parameters::getSamplingMode()
{
  return oss_;
}
