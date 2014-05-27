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

#include "uniserial/uniserial.hpp"

// ROS console output for debugging
#include <ros/console.h>

uniserial::uniserial() :
  buffer_index_( 0 ), // set the buffer index at the head of the serial buffer.
  baud_rate_( 115200 ),
  num_bits_( 8 ),
  parity_( 0 ),
  num_stop_bits_( 1)
{
  // ROS parameter for port name here!
  port_name_ == "/dev/ttyACM0";
}

uniserial::~uniserial()
{
  close( file_descriptor_ );
}

bool uniserial::initialize( const std::string port_name, uint8_t num_bits, uint8_t parity, uint8_t num_stop_bits, uint16_t baud_rate )
{
  struct termios options;
 
  port_name_ = port_name;

  ROS_INFO("uniserial::initialize(): Initializing on defined settings.");
  file_descriptor_ = open( port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
  if( file_descriptor_ == -1 )
  {
    ROS_ERROR("uniserial::initialize(): Unable to open port.");
    return false;
  }
    
  if( tcgetattr( file_descriptor_, &options ) < 0 )
  {
    ROS_ERROR("uniserial::initialize(): Couldn't get terminal attributes.");
    return false;
  }
    
  speed_t b_rate;
    
  switch( baud_rate_ )
  {
  case 4800:
    b_rate = B4800;
    break;
  case 9600:
    b_rate = B9600;
    break;
#ifdef B14400
  case 14400:
    b_rate = B14400;
    break;
#endif
  case 19200:
    b_rate = B19200;
    break;
#ifdef B28800
  case 28800:
    b_rate = B28800;
    break;
#endif
  case 38400:
    b_rate = B38400;
    break;
  case 57600:
    b_rate = B57600;
    break;
  case 115200:
    b_rate = B115200;
    break;
  default:
    b_rate = B115200;
  }
  cfsetispeed( &options, b_rate );
  cfsetospeed( &options, b_rate );

  // 8N1, the default Arduino Serial format: 8-bit, no parity, 1 stop bit.
  
  // Apply parity 
  switch( parity )
  {
  case 0:
    options.c_cflag &= ~PARENB; // no parity.
    break;
  case 1:
    options.c_cflag &= PARODD; // odd parity.
    break;
  case 2:
    options.c_cflag &= PARENB; // even parity (by default).
    break;
  default:
    options.c_cflag &= ~PARENB;
  }
  
  // Apply number of stop bits
  switch( num_stop_bits_ )
  {
  case 1:
    options.c_cflag &= ~CSTOPB;
    break;
  case 2:
    options.c_cflag &= CSTOPB;
    break;
  default:
    options.c_cflag &= ~CSTOPB;
  }

  // disable CSIZE
  options.c_cflag &= ~CSIZE;  
  switch( num_bits )
  {
  case 5:
    options.c_cflag |= CS5;
    break;
  case 6:
    options.c_cflag |= CS6;
    break;
  case 7:
    options.c_cflag |= CS7;
    break;
  case 8:
    options.c_cflag |= CS8;
    break;
  default:
    options.c_cflag |= CS8;
  }
  // disable flow control
  options.c_cflag &= ~CRTSCTS;
  
  // CREAD enables receiver.
  // CLOCAL ignores modem status lines.
  options.c_cflag |= CREAD | CLOCAL;
  // ~IXON disables start/stop output control.
  // ~IXOFF disables start/stop input control.
  // ~IXANY disables any character from restarting output
  options.c_iflag &= ~( IXON | IXOFF | IXANY ); 
  
  // ~ICANON disables canonical input and gives us acces to VMIN and VTIME
  // ~ECHO disables echo.
  // ~ECHOE disables other echoes.
  // ~ISIG disables signals.
  options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG ); // make raw
  options.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  options.c_cc[VMIN]  = 1; // when reading, VMIN is the minimum number of bytes that must arrive before the read function returns.
  options.c_cc[VTIME] = 0; // when reading, VTIME is the maximum interval between bytes in 0.1 second intervals. In this case we have 0.5 second delay between bytes max
    
  /* These next two lines may be system-dependent, 
   * in case it's openning a channel twice-in-a-row.
   * http://todbot.com/blog/2006/12/06/arduino-serial-c-code-to-talk-to-arduino/
   */
  //sleep(1); //Sleep time works only from 1 to 2 (2 not included)
  //tcflush(fd, TCIFLUSH);
    
  if( tcsetattr( file_descriptor_, TCSANOW, &options ) < 0 )
  {
    ROS_ERROR("init_serialport: Couldn't set serial port attributes.");
    return false;
  }
    
  // We must wait about 3 seconds for the Arduino to recover from a reboot upon initializing a serial connection.   
  ROS_INFO( "uniserial: Serial Port initialized." );

  return true;
}

bool uniserial::initialize()
{
  return uniserial::initialize( port_name_, num_bits_, parity_, num_stop_bits_, baud_rate_ );
}

void uniserial::setPortName( const std::string p )
{
  port_name_ = p;
}


uint8_t uniserial::Read()
{
  int bytes_read;
  char temp_buffer;
  bytes_read = read( file_descriptor_, &temp_buffer, 1 );
  if( bytes_read < 0 )
  {
    ROS_ERROR( "Error reading!" );
    return 0;
  }
  return (uint8_t)temp_buffer;
}


bool uniserial::Write( uint8_t byte ) // writes a single byte to the serial device.
{
  short bytes_written;
  char buff_to_write[1];
  buff_to_write[0] = byte;
 
  bytes_written = write( file_descriptor_, buff_to_write, 1 );
  if( bytes_written != 1 )
  {
    return false;
  }
  return true;
}

bool uniserial::Load_Byte( uint8_t byte )
{
  // load  byte into current open index:
  Buffer[buffer_index_] = byte;
  buffer_index_++;
  return true;
}


int uniserial::Available()
{
  int num_bytes;
  ioctl( file_descriptor_, FIONREAD, &num_bytes );
  return num_bytes;
}

bool uniserial::Flush()
{
  int success = tcflush( file_descriptor_, TCIOFLUSH );
  if( success == 0 )
    return true;
  else
    return false;
}

bool uniserial::Read_Bytes( uint8_t num_bytes, uint8_t* storage_buffer )
{
  int8_t bytes_read;
  bytes_read = read( file_descriptor_, storage_buffer, num_bytes );
  if( bytes_read != (int8_t)num_bytes )
    return false;
  else
    return true;
 
}

bool uniserial::Write_Bytes( uint8_t num_bytes, uint8_t* bytes_to_write )
{
  if( num_bytes == 0 )
    return true;
  
  int8_t bytes_written;
  bytes_written = write( file_descriptor_, bytes_to_write, num_bytes );
  if( bytes_written != num_bytes )
    return false;
  else
    return true;
}


 // // An example.  Reads 3 bytes, and writes the byte: 55 to the serial device.
 // int main()
 // {
 //   uniserial Serial;
 //   Serial.setPortName("/dev/ttyACM1");
 //   Serial.initialize();
 //   sleep(3);
 //   int val[3] = { Serial.Read(), Serial.Read(), Serial.Read() };
 //   bool write_status = Serial.Write(55);
 //   if( write_status == false )
 //   {
 //     cout << "writing failed." << endl;
 //   }
 //   cout << "I have read " << val[0]<<" "<< val[1] <<" " << val[2] << endl;
   
 //   sleep(1);
 //   return 0;
 // }
