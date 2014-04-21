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

//\Author Joshua Vasquez and Philip Roan, Robert Bosch LLC

#ifndef ARDUINO_CONSTANTS_H_
#define ARDUINO_CONSTANTS_H_

/** This file includes constants common to the arduino sketches and the driver.
 * It includes bosch_drivers_common.hpp for the \p interface_protocol enumeration
 * and the \p NULL_DEVICE definition.
 */

#include <bosch_drivers_common.hpp>

namespace bosch_drivers_common
{
  /**
   * \brief Default BAUD rate for connecting devices over serial ports
   *
   * Constant is of type \p long to fit on Arduino, where an int is 16 bits.
   */
  static const unsigned long DEFAULT_BAUD_RATE = 115200;

  /**
   * The following constants are serial codes used to ensure that the 
   * Arduino hardware and arduino_interface class are communicating 
   * correctly over the serial port:
   * \var static const uint8_t VERIFY  
   * \var static const uint8_t SUCCESS 
   * \var static const uint8_t FAILURE      
   * \var static const uint8_t FREQ_STANDARD 
   * \var static const uint8_t FREQ_FAST
   */
  static const uint8_t VERIFY  = 0x33; 
  static const uint8_t SUCCESS = 0x34;
  static const uint8_t FAILURE = 0x35;
  
  /**
   * \brief I2C frequency selection 
   */
  
  static const uint8_t FREQ_STANDARD = 0x00; // 100 kHz
  static const uint8_t FREQ_FAST     = 0x01; // 400 kHz

  static const uint8_t READ = 0;
  static const uint8_t WRITE = 1;
  
  /**
   * \brief ADC constants
   */
  uint16_t MAX_ADC_VALUE = 1023;
}

#endif //__ARDUINO_CONSTANTS_H__
