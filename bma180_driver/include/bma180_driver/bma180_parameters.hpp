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


#ifndef BMA180_PARAMETERS_H_
#define BMA180_PARAMETERS_H_

// ROS headers for debugging output
#include <ros/console.h>

#include <bosch_drivers_parameters.hpp>
#include <bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

class BMA180Parameters: public Parameters
{
public:
  
  /**
   * \brief Write these to <3:1> in  ADDRESS_RANGE_REG to change sensitivity
   */
  enum accel_range
  {
    /** \note ±1 [g] at 0.13 [mg/LSB] */
    RANGE_1 = 0x00,
    /** \note ±1.5 [g] at 0.19 [mg/LSB] */
    RANGE_1_5 = 0x01,
    /** \note ±2 [g] at 0.25 [mg/LSB] */  
    RANGE_2 = 0x02,
    /** \note ±3 [g] at 0.38 [mg/LSB] */
    RANGE_3 = 0x03,
    /** \note ±4 [g] at 0.50 [mg/LSB] */
    RANGE_4 = 0x04,
    /** \note ±8 [g] at 0.99 [mg/LSB] */
    RANGE_8 = 0x05,
    /** \note ±16 [g] at 1.98 [mg/LSB] */
    RANGE_16 = 0x06
  };
  
  /**
   * \brief Write these to  <7:4> in ADDRESS_BWTCS to change bandwidth filter
   */
  enum bandwidth 
  {
    BW_10,
    BW_20,
    BW_40,
    BW_75,
    BW_150,
    BW_300,
    BW_600,
    BW_1200,
    BW_HIGH_PASS,
    BW_BAND_PASS
  };
  
  BMA180Parameters();
  ~BMA180Parameters();

  bool setProtocol( interface_protocol protocol );
  bool setFrequency( int frequency );
  bool setPin( uint8_t pin );       
  
  /**
   * \brief Set the driver to the correct device address of the sensor. 
   */
  bool setSlaveAddressBit( bool choice );

  /**
   * \brief Set the sensing range. See \p accel_range
   */
  bool setAccelRange( accel_range new_range ); 
  
  /**
   * \brief Set the internal filter on the sensor to the specified bandwidth.
   */
  void setBandwidth( bandwidth bw );
  void setPreCalOffsets( bool choice );
  
  
  double getSensitivity(); // returns sensitivity
  interface_protocol getProtocol();
  int getFrequency();
  int* getFlags();
  int getPin();  // SPI, GPIO only
  uint8_t getSlaveAddressBit();


protected:
  /**
   * \brief Adjust the byte is the order of the transmission.
   */
  bool setByteOrder( uint8_t value );
  /**
   * \brief Set the SPI mode (0 -- 4).
   */
  bool setSpiMode( uint8_t mode );

  // int frequency_;
  // int pin_;
  // int flags_;
  uint8_t slave_address_bit_;
  double sensitivity_;
  accel_range accel_range_;
  bandwidth bandwidth_;
  bool useFilter_;
  bool offsetsEnabled_;
};

#endif // BMA180_PARAMETERS_H_
