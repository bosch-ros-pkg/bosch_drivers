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

//\Author Joshua Vasquez, Kai Franke, and Philip Roan, Robert Bosch LLC

#ifndef ARDUINO_INTERFACE_H_
#define ARDUINO_INTERFACE_H_

// ROS headers for debugging output
#include <ros/console.h>

#include <bosch_drivers_hardware_interface.hpp>
#include <bosch_drivers_parameters.hpp>

#include <uniserial.hpp>
#include "arduino_constants.hpp"

using namespace bosch_drivers_common;

class ArduinoInterface: public bosch_hardware_interface
{
  /**
   * \brief The arduino hardware interface.
   * Each arduino connected to your computer needs its own instance of this class.
   * The Serial number is the manner by which we differentiate between arduino devices.  
   * 
   * An instance of this class will relay information from the program to the Arduino and vice versa.
   *  Specifically:
   *   inputs: 
   *    communication protocol (only when first initializing a sub20_interface.)
   *    address, 
   *    read or write command to the sensor
   *   outputs: 
   *    raw data (in bytes) from the sensor.
   */

public:
 
  /**
   * \brief constructor
   * \param std::string dev_port the name of the port from the /dev directory.
   */
  ArduinoInterface( std::string dev_port );
  /**
   * \brief destructor
   */
  ~ArduinoInterface();

 
  bool initialize();

  /**
   * \brief Reads \a num_bytes from the requested device on the specified \a protocol at the specified protocol \a frequency
   * \var   int device_address the way that the sensor identifies itself
   * \var   interface_protocol protocol the defined protocol
   * \var   int frequency the frequency of the interface protocol
   * \var   int* flags additional information necessary to read from that particular interface protocol.
   * \var   uint8_t reg_address the starting address of the data in the sensor.
   * \var   uint8_t* data the name of the array where the data will be stored.
   * \var   uint8_t num_bytes the number of bytes to be read from the sensor.
   * \return \a num_bytes or a value less than zero, if the read failed.
   */
  ssize_t read( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes );
  
  /**
   * \brief Writes \a num_bytes from the requested device on the specified \a protocol at the specified protocol \a frequency
   * \var   int device_address the way that the sensor itentifies itself
   * \var   interface_protocol protocol the defined protocol
   * \var   int frequency the frequency of the interface protocol
   * \var   int* flags additional information necessary to write to that particular interface protocol.
   * \var   uint8_t reg_address the starting address in the sensor's registers where the data will be written to.
   * \var   uint8_t* data the name of the array where the data will be output from.
   * \var   uint8_t num_bytes the number of bytes to be written to the sensor.
   * \return \a num_bytes or a value less than zero, if the write failed.
   */
  ssize_t write( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes );

  /**
   * \brief  Returns true if the input protocol is supported by the hardware interface.
   * \param  interface_protocol protocol the input protocol.
   * \return true, if the hardware interface supports reading and writing on that particular protocol.
   */
  bool supportedProtocol( interface_protocol protocol );
  
  /**
   * \return  the way the sensor identifies itself
   */
  std::string getID();

private:

  ssize_t arduinoSpiRead( uint8_t frequency, uint8_t flags, uint8_t reg_address, uint8_t* data, size_t num_bytes );
  ssize_t arduinoSpiWrite( uint8_t frequency, uint8_t flags, uint8_t reg_address, uint8_t* data, size_t num_bytes );
  ssize_t arduinoI2cRead( uint8_t device_address, uint32_t frequency, uint8_t reg_address, uint8_t* data, size_t num_bytes );                
  ssize_t arduinoI2cWrite( uint8_t device_address, uint32_t frequency, uint8_t reg_address, uint8_t* data, size_t num_bytes );
	ssize_t arduinoPwmWrite( uint32_t frequency, uint8_t reg_address, uint8_t data );
	ssize_t arduinoGpioRead( uint8_t flags, uint8_t pin, uint8_t* value );
	ssize_t arduinoGpioWrite( uint8_t pin, bool value );
	ssize_t arduinoEncoderRead( int* pin, uint8_t* data );
	ssize_t arduinoEncoderWrite( int* flags, uint8_t* data );
	ssize_t arduinoAdcWrite( uint8_t* voltage );
	ssize_t arduinoAdcRead( uint8_t pin, uint8_t* data );

  bool waitOnBytes( int num_bytes );

  /**
   * \brief Serial port where the Arduino can be found.
   *
   * This is the serial port that the particular Arduino uses to communicate
   *  with the computer.
   */
  uniserial* serial_port_;  
  std::string port_name_;
 
  /**
   * \brief The file descriptor that handles serial communications.
   */
  int file_descriptor_; 


  /**
   * \brief Baud rate at which the serial communication takes place.
   * \warning This value should not be changed after it is set in the
   * constructor.If it is changed, however, it must also be redefined in the
   * Arduino sketch.
   */
  int baud_rate_;
  
  bool connection_failure_;
  
  /**
   * \brief The time to wait (in seconds) for acknowledgement of a serial command
   *  by the Arduino.
   */
  double timeout_;

  /**
   * \brief True if hardware has already been initialized to prevent re-opening
   *  the serial port.
   */
  bool is_initialized_;
 
  /**
   * \brief Contains used protocol and whether or not a read or write
   * command is about to be requested.
   *
   * This information allows the Arduino to prepare to send the following data
   * according to the input parameters of \a data_packet_ .
   * The read/write flag is saved in the lowest bit while the upper 7 bits
   * are reserved for the used protocol
   */
  uint8_t data_packet_;
  
  /**
   * \brief stores the selected reference voltage for the ADC in milli volts [mV]
   */
  uint16_t _reference_voltage;

};
#endif //ARDUINO_INTERFACE_H_

