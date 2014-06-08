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

#include "sub20_interface/sub20_interface.hpp"

/**********************************************************************/
/**********************************************************************/
Sub20Interface::Sub20Interface( std::string sub20_serial_number ) :
  serialNumber_( sub20_serial_number ),
  num_devices_( 0 ),
  subdev_( NULL ),
  is_initialized_( false )
{
  // make sure serialNumber_ is at least 10 chars
}


/**********************************************************************/
/**********************************************************************/
Sub20Interface::~Sub20Interface()
{
}


/**********************************************************************/
/**********************************************************************/
bool Sub20Interface::initialize()
{
  /* To handle a setup with multiple sub20s where One or more of them is 
   * unplugged, we may want to rewrite the search routine so 
   * that it stores all the sub20s to a list and iterates through them
   */
  if( is_initialized_ == true )
    return true;

   error_code = 0;
  bool device_found = false;

  while( device_found == false )
  {
    // scan for sub20s, starting at the first.
    //ROS_INFO("Searching for sub20(%s).", serialNumber_);  

    while( subdev_ = sub_find_devices( subdev_ ) )
    { // get a handle on that sub20
      handle_ = sub_open( subdev_ );

      if( handle_ == NULL )
      {
	// if there's an error trying to get a handle on that sub20, keep going!  That one has probably already been setup.
	ROS_WARN("Sub20Interface::initialize(): sub_open status: %s", sub_strerror(sub_errno)); 
      }
      else  // FOUND an unused sub20 plugged in:
      {
	// get the current sub20's serial number. Put it in the buffer.
	char found_serial_number[11];
	sub_get_serial_number( handle_, found_serial_number, 10 );
	//ROS_INFO("Found: sub20(%s).", found_serial_number );
    
	// if no serial number was specified in the constructor:
	if( strcmp( "null", found_serial_number ) == 0 )
	{
	  // make the discovered serial number the class's serial number:
	  serialNumber_ = std::string( found_serial_number );
	  device_found = true;
	  break;
	}
    
	// if a serial number was specified, check if it's the right sub20:
	if( strcmp( found_serial_number, serialNumber_.c_str() ) == 0)
	{
	  device_found = true;
	  break;
	}
      }
      // Otherwise scan for the next sub20 up at the top
    }
 
    if( device_found == true )
      break;
   
    // TWO possibilities for failure:
    // 1. if no sub20s are available:
    if( strcmp( "null", serialNumber_.c_str() ) == 0 )
    {
      ROS_ERROR("Sub20Interface::initialize() Couldn't find any sub20s plugged in.");
      return false;
    }
    // 2. if we've looked through all sub20s, and none have the matching serial number:
    ROS_ERROR("Sub20Interface::initialize() Couldn't find sub20 [serial: \"%s\"]. ", serialNumber_.c_str() );  
    return false;
  }
  // SUCCESS:
  ROS_INFO("Sub20Interface::initialize() sub20 [serial: \"%s\"] initialization complete.", serialNumber_.c_str() );
  is_initialized_ = true;
  return true;
}


/**********************************************************************/
/**********************************************************************/
ssize_t Sub20Interface::read( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  int error_code = 0;

  switch( protocol )
  {
  case I2C:
  {
    // configure i2c bus according to parameters:
    if( sub_i2c_freq( handle_, &frequency ) < 0 )
    {
      ROS_ERROR("Sub20Interface: I2c setup failed. \r Status: %s", sub_strerror(sub_errno) );
      return -1;
    }
    if( sub_i2c_read( handle_, device_address, reg_address, 1, (char*)data, num_bytes ) < 0 )
    {
      ROS_ERROR("Sub20Interface: I2c read failed. \r Status: %s", sub_strerror(sub_errno));
      return -1;
    }
    break;
  }
  case SPI:
  {
    // configure SPI bus according to parameters:
    if( spiConfigRoutine( frequency, *flags ) == false )
    {
      ROS_ERROR("Sub20Interface::read(...): SPI configuration failed.");
      return -1;
    }

    // SPI Reading has extra overhead:
    //CREATE OUTPUT DATA ARRAY: The output data is an array with reg address followed by num_bytes of dud values.
    uint8_t output_data[num_bytes + 1];
    output_data[0] = reg_address;
  
    for( int i = 0; i < (int)num_bytes; i++ )
    {
      output_data[i+1] = (uint8_t)0xFF;
    }
  
    // CREATE INPUT DATA ARRAY:
    uint8_t input_data[num_bytes + 1];
  
    // BEGIN transfer: two variations, depending on device address
    switch( device_address )
    {
    case NULL_DEVICE:
      // Transfer without toggling chip_select line:
      if( sub_spi_transfer( handle_, (char*)output_data, (char*)input_data, (num_bytes + 1), SS_CONF(device_address, SS_H) ) < 0)
      {
	ROS_ERROR( "Sub20Interface::read(...): SPI read failed. \r Status: %s", sub_strerror(sub_errno) );
	return -1;
      }
      break;
    default:
      // Transfer normally (pull chip_select line low):
      if( sub_spi_transfer( handle_, (char*)output_data, (char*)input_data, (num_bytes + 1), SS_CONF(device_address, SS_LO) ) < 0)
      {
	ROS_ERROR( "Sub20Interface::read(...): SPI read failed. \r Status: %s", sub_strerror(sub_errno) );
	return -1;
      } 
    }
  
    // Transfer reg_address, and then receive num_bytes of data back:
    if( sub_spi_transfer( handle_, (char*)output_data, (char*)input_data, (num_bytes + 1), SS_CONF(device_address, SS_LO) ) < 0)
    {
      ROS_ERROR("Sub20Interface::read(...): SPI read failed. \r Status: %s", sub_strerror(sub_errno));
      error_code = (-1);
      return error_code;
    }
  
    // Copy input_data into data:
    for( int j = 0; j < (int)num_bytes; j++ )
    {
      data[j] = input_data[j+1];
    }
  
    break;
  }
  case GPIO:
  case RS232:
    ROS_ERROR("Sub20Interface: Driver does not support this protocol at this time.");
    break;
  default:
    ROS_ERROR("Sub20Interface: Invalid protocol for Sub20.");
    return 1;
  }
  
  return error_code;
}


/**********************************************************************/
/**********************************************************************/
ssize_t Sub20Interface::write( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  int error_code = 0;
  
  switch( protocol )
  {
  case I2C:
  {
    if( sub_i2c_freq(handle_, &frequency) < 0 )
    {
      ROS_ERROR("Sub20Interface::write(...): ERROR.  Could not set I2C frequency.");
      return -1;
    }
    if( sub_i2c_write( handle_, device_address, reg_address, 1, (char*)data, num_bytes ) < 0 )
    {
      ROS_ERROR( "Sub20Interface::write(...): ERROR. Could not write data over I2C interface." );
      return -1;
    }
    break;
  }
  case SPI:
  {
    if( spiConfigRoutine( frequency, *flags ) == false )
    {
      ROS_ERROR("Sub20Interface::write(...): SPI configuration failed.");
      return -1;
    }

    // SPI writing has extra overhead:  
    //CREATE OUTPUT DATA ARRAY:                     The output data is an array with reg address followed by the data.
    uint8_t output_data[num_bytes + 1];
    output_data[0] = reg_address;
  
    for( int i = 0; i < (int)num_bytes; i++ )
    {
      output_data[i+1] = data[i];
    }
    // transfer reg_address, then data:
    error_code += sub_spi_transfer( handle_, (char*)output_data, 0, (num_bytes+1), SS_CONF(device_address, SS_LO) ); //VERIFY SS_LO
    return error_code;
  }
  case GPIO:
  case RS232:
    ROS_ERROR("Sub20Interface: Driver does not support this protocol at this time.");
    break;
  default:
    ROS_ERROR("Sub20Interface: Invalid protocol for Sub20.");
    return -1;
  }
  
  return num_bytes;
}


/**********************************************************************/
/**********************************************************************/
bool Sub20Interface::supportedProtocol( interface_protocol protocol )
{
  switch( protocol )
  {
  case GPIO:
    return false;
  case RS232:
    return false;
  case SPI:
  case I2C:
    return true;
  default:
    return false;
  }
}


/**********************************************************************/
/**********************************************************************/
std::string Sub20Interface::getID()
{
  return serialNumber_;
}


/**********************************************************************/
/**********************************************************************/
int Sub20Interface::numDevices()
{
  return num_devices_;
}


/**********************************************************************/
/**********************************************************************/
bool Sub20Interface::spiConfigRoutine( int frequency, int flags )
{
  int spi_configuration_ = SPI_ENABLE;
 
  // set frequency:
  switch( frequency )
  {
  case 8000000:
    spi_configuration_ |= SPI_CLK_8MHZ;
    break;
  case 4000000:
    spi_configuration_ |= SPI_CLK_4MHZ;
    break;
  case 2000000:
    spi_configuration_ |= SPI_CLK_2MHZ;
    break;
  case 1000000:
    spi_configuration_ |= SPI_CLK_1MHZ;
    break;
  case 500000:
    spi_configuration_ |= SPI_CLK_500KHZ;
    break;
  case 250000:
    spi_configuration_ |= SPI_CLK_250KHZ;
    break;
  case 125000: 
    spi_configuration_ |= SPI_CLK_125KHZ;
    break;
  default:
    ROS_ERROR( "Sub20Interface::spiConfigRoutine(): Invalid Sub20 frequency." );
    return false;
  }
 
  // decrypt flags, which were set in the parameters class:
  uint8_t bit_order = ( (flags>>2) & 0x01 );  //0x01 = B00000001
  uint8_t mode = (flags & 0x03);        //0x03 = B00000011
  uint8_t spi_slave_select = ((flags >> 4) & 0x0F); //0x0F = B00001111
  
    // set bit-order:
  switch( bit_order )
  {
  case 0:
    spi_configuration_ |= SPI_LSB_FIRST;
    break;
  case 1:
    spi_configuration_ |= SPI_MSB_FIRST;
    break;
  default:
    ROS_ERROR("Sub20Interface: Invalid Sub20 bit-order.");
    return false;
  }
   
  // set CPOL and CPHA (identical to Arduino flags)
  switch( mode )
  {
  case 0:
    spi_configuration_ |= SPI_CPOL_RISE | SPI_SMPL_SETUP;       // CPOL = 0; CPHA = 0
    break;
  case 1:
    spi_configuration_ |= SPI_CPOL_RISE | SPI_SETUP_SMPL;       // CPOL = 0; CPHA = 1
    break;
  case 2:
    spi_configuration_ |= SPI_CPOL_FALL | SPI_SMPL_SETUP;       // CPOL = 1; CPHA = 0
    break;
  case 3:
    spi_configuration_ |= SPI_CPOL_FALL | SPI_SETUP_SMPL;       // CPOL = 1; CPHA = 1
    break;
  default:
    // impossible, since mode is a two-bit value.
    ROS_ERROR("Sub20Interface: Invalid Sub20 CPOL and CPHA settings.");
    return false;
  }
  
  // send configuration to the Sub20:
  if( sub_spi_config( handle_, spi_configuration_, 0 ) < 0 )
  {
    ROS_ERROR("Sub20Interface: SPI setup failed. \r Status: %s", sub_strerror(sub_errno)); 
    return false;
  }
  
  // verify that the spi_slave_select is valid:
  switch( spi_slave_select )
  {
  case 0:
    break;
  case 1:
    break;
  case 2:
    break;
  case 3:
    break;
  case 4:
    break;
  default:
    ROS_ERROR("Sub20Interface: Invalid slave select.  Choose a slave on pins 0-4.");
    return false;
  }
  
  return true;
}
