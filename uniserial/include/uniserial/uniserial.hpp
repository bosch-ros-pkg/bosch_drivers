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


#ifndef UNISERIAL_H_
#define UNISERIAL_H_

#include <stdlib.h> 
#include <stdint.h>   /* standard types */
#include <string.h>   /* string function defs */
#include <unistd.h>   /* UNIX's standard function defs */
#include <fcntl.h>    /* file control function defs */
#include <errno.h>    /* error number definitions */
#include <termios.h>  /* POSIX terminal control function defs */
#include <sys/ioctl.h>
#include <getopt.h>
#include <iostream>

/** a simple C++ Serial class for interfacing the Arduino 
 * (or other serial devices) over a Linux-based serial port 
 * using UNIX libraries.
 * 
 * Default settings established in the constructor will connect with an 
 * Arduino device at 115200 baud.
 */
class uniserial
{
public:
  uniserial();
  ~uniserial();

  bool initialize(); // initialize under defaults.
  bool initialize( const std::string port_name, uint8_t num_bits, uint8_t parity, uint8_t num_stop_bits, uint16_t baud_rate );
  void setPortName( const std::string p ); // sets the name of the port
  uint8_t Read(); // returns the next byte. Read one byte into a temporary buffer and return the value at that buffer.
  bool Write( uint8_t byte ); // writes the array to the arduino.
  int Available(); // returns number of bytes currently on the serial buffer. 
  bool Flush(); // flushes input and output buffers
    
  bool Load_Byte( uint8_t byte ); // loads byte onto the next spot in the write buffer.
  bool Read_Bytes( uint8_t num_bytes, uint8_t* storage ); // read bytes from Serial port and output to storage.
  bool Write_Bytes( uint8_t num_bytes, uint8_t* bytes_to_write ); // write num_bytes from bytes_to_write
         
private:
  char Buffer[256]; // for storing data from the read() method.
  std::string port_name_; // the name of the serial port.
  int file_descriptor_; // the file descripter used by the unix functions.
  uint8_t buffer_index_; // the buffer's current pointer.
  int baud_rate_; // the communication baud rate. (it must agree with the hardware device's baud rate as well!)
 
  // Hardware dependent settings. Arduino defaults to 8-bit, No parity, 1 stop-bit.
  uint8_t num_bits_;
  uint8_t parity_;
  uint8_t num_stop_bits_;

};

#endif // UNISERIAL_H_
