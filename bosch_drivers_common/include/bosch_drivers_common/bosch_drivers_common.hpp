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

#ifndef BOSCH_DRIVERS_COMMON_H_
#define BOSCH_DRIVERS_COMMON_H_

#include <stdint.h>


/** 
 * Namespace common to all Bosch drivers
 *
 * These values are used repeatedly by multiple files, so I've
 * encapsulated them into their own namespace.
 * The sensor_driver class, parameters class, and
 * bosch_hardware_interface class are in this namespace.  
 */
namespace bosch_drivers_common
{
  /**
   * \brief the name of the compatible hardware devices that the sensors can implement as a generic hardware interface.
   */
  enum Hardware_Interface { Sub20, Arduino, ArduinoROSSerial, Gumstix };
        

  /**
   * \brief the name of the protocol that both the hardware interface and the sensor are using to transmit data.
   *
   * This enumeration is used on the Arduino and on the host PC. To optimize the
   * arduino code, compile with the -fshort-enums flag.
   */
  enum interface_protocol { I2C, SPI, GPIO, RS232, RS485, ETHERNET, ETHERCAT, CAN, JTAG, PROFIBUS, MODBUS, USB, PWM, ENCODER, ADCONVERTER };
  

        
  /**
   * \brief SPI Constants
   *
   * These values are used to set the SPI mode and frequency. Information on SPI
   * modes can be found on the wikipedia page for SPI. The SPI frequency is
   * determined by the clock frequency of the hardware interface and the clock
   * divider value. See the datasheet for the specific hardware interface for
   * additional details. The values here are optimized for use with Arduinos.
   */
  static const uint8_t SPI_MODE_0 = 0x00;
  static const uint8_t SPI_MODE_1 = 0x01;
  static const uint8_t SPI_MODE_2 = 0x02;
  static const uint8_t SPI_MODE_3 = 0x03;
  
  static const uint8_t MSB_FIRST = 1;
  static const uint8_t LSB_FIRST = 0;

  static const uint8_t SPI_CLOCK_DIV_2   = 0x04;  
  static const uint8_t SPI_CLOCK_DIV_4   = 0x00;
  static const uint8_t SPI_CLOCK_DIV_8   = 0x05;
  static const uint8_t SPI_CLOCK_DIV_16  = 0x01;
  static const uint8_t SPI_CLOCK_DIV_32  = 0x06;
  static const uint8_t SPI_CLOCK_DIV_64  = 0x02;
  static const uint8_t SPI_CLOCK_DIV_128 = 0x03;
  //static const uint8_t SPI_CLOCK_DIV64 0x07;

  
  /**
   *  SPI Read/Write Flags
   * \brief  The following members are the relevant bitflag offsets necessary for prompting the sensor for an SPI read or write.
   *
   * \note   These flags are prepended to the register address.
   *
   * \note   These flags are only relevant to sensors that implement SPI
   * communication.
   * \var    static const uint8_t SPI_READ_FLAG
   * \var    static const uint8_t SPI_WRITE_FLAG
   */
  static const uint8_t SPI_READ_FLAG = 7;
  static const uint8_t SPI_WRITE_FLAG = 7;
  
  
  // \brief This constant is used to specify an SPI device with no chip select.
  static const uint8_t NULL_DEVICE = 0xFF;

  /**
   * \brief GPIO Constants
   *
   */
  enum gpio_input_mode { FLOATING, PULLUP, PULLDOWN };

  /**
   * \brief Motor Constants
   *
   */
  enum motor_drive_mode { DRIVE, FREE_RUNNING, BRAKE };

  /**
   * \brief encoder Constants
   *
   */
  enum encoder_control { CREATE, DESTROY, SET_POSITION };
}
#endif // BOSCH_DRIVERS_COMMON_H_
