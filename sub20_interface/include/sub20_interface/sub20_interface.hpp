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

#ifndef SUB20_INTERFACE_H_
#define SUB20_INTERFACE_H_

// ROS headers for debugging output
#include <ros/console.h>

#include <libsub20/libsub.h>
#include <bosch_drivers_common/bosch_drivers_common.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>


using namespace bosch_drivers_common;

/**
 * \brief The sub20 hardware interface.
 *
 * Each sub20 connected to your computer needs its own instance of this class!
 * The Serial number is the manner by which we differentiate between
 * sub20 devices.  
 * 
 *  An instance of this class will relay information from the 
 * program to the sub20 and vice versa.
 *  Specifically:
 *   inputs: 
 *    communication protocol (only when first initializing a sub20_interface.)
 *    address, 
 *    read or write command to the sensor
 *   outputs: 
 *    raw data (in bytes) from the sensor.
 * NOTE: You can initialize multiple sensors on the same sub20 
 * by calling this method for each sensor's device_address.
 * There are some constraints: devices can't have the same
 * address.   
 * 
 */
class Sub20Interface: public bosch_hardware_interface
{

public:
  /**
   * \brief constructor, which takes in an optional argument, which is the
   *     sub20's 4-digit serial number.
   *
   * \param std::string sub20_serial_number
   * \note  if no argument is passed to the constructor, the constructor 
   *     will simply get a handle on the first sub20 it finds that is
   *     plugged in. Furthermore, this method prints the serial number 
   *     to the screen.  This is an excellent way of finding out the 
   *     4-digit serial number of your sub20 if you do not already
   *     know it.
   * \note  Knowing the sub20 serial number is only relevant for a sensor 
   *     configuration that involves multiple sub20s connected to 
   *     the same computer.
   * \warning the string \a "null" is a magic value also used in the cpp
   *     file because static const std::strings cannot be 
   *     created in headers.
   */
  Sub20Interface( std::string sub20_serial_number =  "null" ); 

  /**
   * \brief destructor
   */
  ~Sub20Interface(); // destructor
  

  /**
   * \brief finds the sub20 specified by the 4-digit serial in the
   *     constructor (or the first sub20, if no serial is specified)
   *     and establishes a connection with it.
   */ 
  bool initialize();

  /**
   * \brief  reads \a num_bytes from the requested device on the 
   *      specified \a protocol at the specified protocol \a frequency
   *
   * \param   int device_address the way that the sensor identifies itself
   * \param   interface_protocol protocol the defined protocol
   * \param   int frequency the frequency of the interface protocol
   * \param   int* flags additional information necessary to read from
   *      that particular interface protocol.
   * \param   uint8_t reg_address the starting address of the data in the
   *      sensor.
   * \param   uint8_t* data the name of the array where the data will
   *      be stored.
   * \param   uint8_t num_bytes the number of bytes to be read from the
   *      sensor.
   * \return \a num_bytes or a value less than zero, if the read failed.
   */
  ssize_t read( uint8_t device_address,
		interface_protocol protocol,
		unsigned int frequency,
		uint8_t flags,
		uint8_t register_address,
		uint8_t* data,
		size_t num_bytes );

  ssize_t read( bosch_drivers_communication_properties communication_properties,
		uint8_t register_address, 
		std::vector<uint8_t> &data );


  /**
   * \brief  Writes \a num_bytes from the requested device on the 
   *      specified \a protocol at the specified protocol \a frequency
   *
   * \param   int device_address the way that the sensor itentifies itself
   * \param   interface_protocol protocol the defined protocol
   * \param   int frequency the frequency of the interface protocol
   * \param   int* flags additional information necessary to read from
   *      that particular interface protocol.
   * \param   uint8_t reg_address the starting address in the sensor's
   *      registers where the data will be written to.
   * \param   uint8_t* data the name of the array where the data will
   *      be output from.
   * \param   uint8_t num_bytes the number of bytes to be written to the
   *      sensor.
   * \return \a num_bytes or a value less than zero, if the read failed.
   */
  ssize_t write( uint8_t device_address,
		 interface_protocol protocol,
		 unsigned int frequency,
		 uint8_t flags,
		 uint8_t register_address,
		 uint8_t* data,
		 size_t num_bytes );

  ssize_t write( bosch_drivers_communication_properties device_communication_properties,
		 uint8_t register_address, 
		 std::vector<uint8_t> data );

  /**
   * \brief  returns true if the input protocol is supported by the
   *      hardware interface.
   *
   * \param  interface_protocol protocol the input protocol.
   * \return  true, if the hardware interface supports reading and writing
   *      on that particular protocol.
   */
  bool supportedProtocol( interface_protocol protocol );
  
  /**
   * \return  the way the sensor identifies itself.
   */
  std::string getID();

  /**
   * \return  the number of devices connected via this particular 
   *      sub20 instance.
   */
  int numDevices();

private:
  bool spiConfigRoutine( unsigned int frequency, uint8_t flags );
  
  interface_protocol protocol_;
  int error_code;
  sub_handle handle_; //! sub20 device handle
  sub_device subdev_;
  std::string serialNumber_;
  int num_devices_;
  int spi_configuration_;
  bool is_initialized_;
};

#endif //SUB20_INTERFACE_H_
