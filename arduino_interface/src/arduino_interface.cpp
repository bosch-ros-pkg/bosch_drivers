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

#include "arduino_interface/arduino_interface.hpp"

/**********************************************************************/
// Constructor
/**********************************************************************/
ArduinoInterface::ArduinoInterface( std::string port_name ) :
  port_name_( port_name ),
  baud_rate_( DEFAULT_BAUD_RATE ),
  connection_failure_( true ), // no connection established yet.
  timeout_( 0.5 ), // delay in seconds before we declare Serial communication lost.
  is_initialized_( false ),
  data_packet_( 0 ),
  _reference_voltage(5000)  // set adc reference voltage to 5V, this is Arduino Uno specific
{
  // Create a serial port and assign it the class name: 
  serial_port_ = new uniserial();
}


/**********************************************************************/
// Destructor
/**********************************************************************/
ArduinoInterface::~ArduinoInterface()
{
  delete serial_port_;
}


/**********************************************************************/
// Initialize:   
/**********************************************************************/
bool ArduinoInterface::initialize()
{
  // Prevent Double-initialization!
  if( is_initialized_ == true )
  {
    ROS_INFO( "ArduinoInterface::initialize(): hardware already initialized." );
    return true;
  }
 
  // uniserial init args: [8-bit data], [no parity], [1 stop bit], [class baud rate]. 
  bool init_success = serial_port_->initialize( port_name_, 8 , 0, 1, baud_rate_ );
 
  // At least a 3-second delay while the Arduino resets. Note: We can remove this 3 second startup if we cut the solder bridge on the board's RESET EN trace. 
  sleep( 3 );
 
  // Error if connection failure:
  if( init_success != true )
  {
    ROS_ERROR( "Arduino Serial connection not initialized properly.  Make sure serial_port and baud_rate match on both ends!" );
    connection_failure_ = true;
    return false;
  }
 
  // successful Serial connection!
  connection_failure_ = false; 
  is_initialized_ = true;
  ROS_INFO( "ArduinoInterface::initialize(): hardware initialized." );
  return true;
}


/**********************************************************************/
// Read
/**********************************************************************/
ssize_t ArduinoInterface::read( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  int error_code = 0;
  
  // Check connection:
  if( connection_failure_ == true )
  {
    return -1;
  }
  
  // Specify data package as read command
  data_packet_ = READ;

	// Specify data package protocol
	data_packet_ |= (protocol << 1);
 
  switch( protocol )
  {
    case I2C:
    {
      error_code = arduinoI2cRead( device_address, frequency, reg_address, data, num_bytes );               
      break;
    }
    case SPI: 
    {
      error_code = arduinoSpiRead( (uint8_t)frequency, (uint8_t)flags[0], reg_address, data, num_bytes );
      break;
    } 
    case GPIO:
    {
      error_code = arduinoGpioRead( (uint8_t)flags[0], reg_address, data );
      break;
    }
    case ENCODER:
    {
      error_code = arduinoEncoderRead( flags, data );
      break;
    }
    case ADCONVERTER:
    {
      error_code = arduinoAdcRead( reg_address, data );
      break;
    }
    default:
    {
      ROS_ERROR("Arduino does not support reading through this protocol.");
      return -1;
    }
  }
  return error_code;
}


/**********************************************************************/
// Write
/**********************************************************************/
ssize_t ArduinoInterface::write( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{ 
  int error_code = 0;

  // Check connection:
  if( connection_failure_ == true )
  {
		std::cout << "connection_failure, hardware initialized?" << '\n';
    return -1;
  }
  
	// Specify data package as write command
  data_packet_ = WRITE;

	// Specify data package protocol
	data_packet_ |= (protocol << 1);
 
  switch( protocol )
  {
    case I2C:
	  {
      error_code = arduinoI2cWrite( (uint8_t)device_address, (uint32_t)frequency, reg_address, data, num_bytes );
      break;
	  }
    case SPI:
    {
      error_code = arduinoSpiWrite ( (uint8_t)frequency, (uint8_t)flags[0], reg_address, data, num_bytes );
      break;
    }
	  case PWM:
	  {
	    // Arduino only accepts 8 Bit PWM, so pass MSB only
		  error_code = arduinoPwmWrite( (uint32_t)frequency, reg_address, data[0] );
		  break;
	  }
	  case GPIO:
	  {
      error_code = arduinoGpioWrite( reg_address, (bool)data[0] );
      break;
    }
    case ENCODER:
    {
      error_code = arduinoEncoderWrite( flags, data );
      break;
    }
    case ADCONVERTER:
    {
      error_code = arduinoAdcWrite( data );
      break;
    }
    default:
    {
      ROS_ERROR( "Arduino does not support writing through this protocol." );
      error_code = -1;
    }
  }
 
  return error_code; // bytes written, or error. 
}


/**********************************************************************/
// supportedProtocol
/**********************************************************************/
bool ArduinoInterface::supportedProtocol( interface_protocol protocol )
{
  switch( protocol )
  {
  case SPI: 
    return true;
  case I2C: 
    return true;
  case PWM:
    return true;
  case GPIO:
    return true;
  case ENCODER:
    return true;
  case ADCONVERTER:
    return true;
  case RS232: {}
  case RS485: {}
  case ETHERNET: {}
  case ETHERCAT: {}
  default:
    return false;
  }
}


/**********************************************************************/
std::string ArduinoInterface::getID()
/**********************************************************************/
{
  return port_name_;
}


/**********************************************************************/
/**********************************************************************/
bool ArduinoInterface::waitOnBytes( int num_bytes )
{
  // Error checking variables 
  unsigned int sleep_interval_us = 25;
  unsigned int loop_counter = 0;

  while( serial_port_->Available() < num_bytes ) // wait for Arduino's response.
  {
    double time_lapsed = (double)(loop_counter * sleep_interval_us ) / 1e6; // [s]
    if( time_lapsed > timeout_ )
    {
      ROS_ERROR( "Serial communication timed out waiting for: %d byte(s).", num_bytes );
      connection_failure_ = true;
      return false;
    }
    loop_counter++;
    usleep( sleep_interval_us );
  }
  return true;   
}


/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoI2cWrite( uint8_t device_address, uint32_t frequency, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
	// the only flag for I2C is the frequency, so no external argument accepted
	uint8_t flags;

  // Encode i2c frequency data into flags
	// lower 3 Bits are reserved for frequency
  switch( frequency )
  {
		case 100000:
			flags = FREQ_STANDARD;
		  break;
		case 400000:
			// set Bit 2
		  flags = FREQ_FAST;
		  break;
		default:
		  ROS_ERROR("Arduino cannot write at this frequency.");
		  return -1; // error code. 
  }
   
  uint8_t i2c_write_prompt[5] = { data_packet_, flags, (uint8_t)device_address, reg_address, num_bytes };
  serial_port_->Write_Bytes( 5, i2c_write_prompt );
   
  //Wait for verification:
  waitOnBytes( 1 );
   
  int verification = serial_port_->Read();
   
  // Flush the buffer before receiving the data:
  serial_port_->Flush();
  if( verification != VERIFY )
  {
    ROS_INFO("No response to i2c write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }

  serial_port_->Write_Bytes( num_bytes, data );

  // wait for Arduino's Verification code:
  waitOnBytes( 1 );
   
  // Verify:
  verification = serial_port_->Read();
  if( verification != SUCCESS )
  {
    ROS_ERROR("arduinoI2cWrite error.  No verification.");
    return -1;
  }
   
  return num_bytes;
}


/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoSpiWrite( uint8_t frequency, uint8_t flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  // construct array to send to Arduino:
  uint8_t write_packet[num_bytes + 5];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = frequency;
  write_packet[2] = flags;
  write_packet[3] = reg_address;
  write_packet[4] = num_bytes;

  for( uint8_t i = 0; i < num_bytes; i++ )
  {
    write_packet[i+5] = data[i];
  }
   
  // send the data:
  serial_port_->Write_Bytes( (num_bytes + 5), write_packet );
   
  usleep( 5000 );  
    
  //Wait for verification:
  waitOnBytes( 1 );
   
  uint8_t verification = serial_port_->Read();
   
  //serial_port_->Flush();
  if( verification != VERIFY )
  {
    ROS_INFO("No response to spi write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }
   
  return num_bytes;
}


/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoSpiRead( uint8_t frequency, uint8_t flags, uint8_t reg_address, uint8_t* data, size_t num_bytes ) 
{
  uint8_t spi_read_prompt[5] = { data_packet_, frequency, flags, reg_address, num_bytes };
                
  serial_port_->Write_Bytes( 5, spi_read_prompt );
   
  // wait for the Arduino to verify these commands:
  while( serial_port_->Available() < 1 )
    ; // do nothing until verification arrives.
   
  // Verify:
  int verification = serial_port_->Read();

  if( verification != VERIFY )
  {
    ROS_INFO( "No response to SPI read prompt. Instead:  0x%x", verification );
    return -1; // error code.
  }

  // Timeout Check Routine: 
  bool error = waitOnBytes( num_bytes );
  if( error == false )
  {
    ROS_ERROR( "Read broke: Arduino did not return SPI data." );
    return -1; // error code.
  }

  // Read num_bytes bytes off the serial line:
  if( serial_port_->Read_Bytes( num_bytes, data ) == false )
	{
		ROS_ERROR("Read_Bytes Error");
    return -1; // error code.
	}
  else
    return num_bytes;
}



/**********************************************************************/
ssize_t ArduinoInterface::arduinoI2cRead(uint8_t device_address, uint32_t frequency, uint8_t reg_address, uint8_t* data, size_t num_bytes ) 
{

// the only flag for I2C is the frequency, so no external argument accepted
	uint8_t flags;

  // Encode i2c frequency data into flags
	// lower 3 Bits are reserved for frequency
  switch( frequency )
  {
		case 100000:
			flags = FREQ_STANDARD;
		  break;
		case 400000:
			// set Bit 2
		  flags = FREQ_FAST;
		  break;
		default:
		  ROS_ERROR("Arduino cannot read at this frequency.");
		  return -1; // error code. 
  }

  uint8_t i2c_read_prompt[5] = { data_packet_, flags, (uint8_t)device_address, reg_address, num_bytes };
                
  serial_port_->Write_Bytes( 5, i2c_read_prompt );  
   
  // wait for the Arduino to verify these commands:
  while( serial_port_->Available() < 1 )
    ;

  // Verify:
  int verification = serial_port_->Read();

  if( verification != VERIFY )
  {
    ROS_ERROR("ArduinoInterface::arduinoI2cRead(): No response to I2C read prompt. Instead:  0x%x", verification);
    return -1; // error code.
  }
   
  // Wait for data to arrive from sensor:
  //Timeout Check Routine:   
  bool error = waitOnBytes( num_bytes );
  if( error == false )
  {
    ROS_ERROR("ArduinoInterface::arduinoI2cRead(): Read broke: did not receive data back.");
    return -1; // error code.
  } 
   
  // Read num_bytes bytes off the serial line:
  if( serial_port_->Read_Bytes( num_bytes, data) == false )
    return -1; // error code.
  else
    return num_bytes;
}

/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoPwmWrite( uint32_t frequency, uint8_t reg_address, uint8_t data )
{
	if( frequency != 490)
	{
		ROS_ERROR("Only frequency 490Hz suppported for Arduino");
		return -1;
	}
	/* 
	 * the following block is Arduino Uno specific
	 */
	switch (reg_address)
	{
	  case 3:
	  case 5:
	  case 6:
	  case 9:
	  case 10:
	  case 11: break;
	  default:
	  {
	    ROS_ERROR("The selected Pin number (reg_address) is not available for PWM");
	    ROS_ERROR("Select Pins 3,5,6,9,10,11 instead");
	    return -1;
	  }
	}
	
  // construct array to send to Arduino:
  uint8_t write_packet[3];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = reg_address;
	write_packet[2] = data;
   
  // send the data:
  serial_port_->Write_Bytes( 3, write_packet );
   
  usleep( 5000 );    
  //Wait for verification:
  waitOnBytes( 1 );
   
  uint8_t verification = serial_port_->Read();
   
  if( verification != SUCCESS )
  {
    ROS_INFO("No SUCCESS response to PWM write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }
   
  return 1; // wrote one byte
}

/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoGpioWrite( uint8_t pin, bool value )
{
  /* 
	 * the following block is Arduino Uno specific
	 */
  switch (pin)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13: break;
    default:
    {
	    ROS_ERROR("The selected Pin number is not available for GPIO");
	    ROS_ERROR("Select Pins 0 through 13 instead");
	    return -1;
	  }
  }
  // construct array to send to Arduino:
  uint8_t write_packet[3];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = pin;
	write_packet[2] = value;
	
  // send the data:
  serial_port_->Write_Bytes( 3, write_packet );
  usleep( 5000 );    
  //Wait for verification:
  if(!( waitOnBytes( 1 )))
    return -1;
   
  uint8_t verification = serial_port_->Read();
   
  if( verification != SUCCESS )
  {
    ROS_INFO("No SUCCESS response to GPIO write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }
  return 1;
}

/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoGpioRead( uint8_t flags, uint8_t pin, uint8_t* value )
{
  /* 
	 * the following block is Arduino Uno specific
	 */
  switch (pin)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13: break;
    default:
    {
	    ROS_ERROR("The selected Pin number is not available for GPIO");
	    ROS_ERROR("Select Pins 0 through 13 instead");
	    return -1;
	  }
  }
  
  switch ((gpio_input_mode) flags )
  {
    /* 
	   * PULLUP will only work with Arduino version 1.0.1 or greater!!!
	   */
    case FLOATING: 
    case PULLUP: break;
    case PULLDOWN:
    {
      ROS_ERROR("The selected input mode is not available for Arduino");
	    ROS_ERROR("Select FLOATING instead");
	    return -1;
    }
    default:
    {
      ROS_ERROR("ArduinoInterface::arduinoGpioRead The selected input mode is not known");
      return -1;
    }
  }
  // construct array to send to Arduino:
  uint8_t write_packet[3];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = flags;
	write_packet[2] = pin;
   
  // send the data:
  if(! serial_port_->Write_Bytes( 3, write_packet ))
  {
    ROS_ERROR("ArduinoInterface::arduinoGpioRead(): Could not send data to Arduino");
    return -1;
  }
   
  usleep( 5000 );    
   
  // Wait for data to arrive from sensor:
  //Timeout Check Routine:   
  bool error = waitOnBytes( 1 );
  if( error == false )
  {
    ROS_ERROR("ArduinoInterface::arduinoGpioRead(): Read broke: did not receive data back.");
    return -1; // error code.
  } 
  // Read value off the serial line:
  if( serial_port_->Read_Bytes( 1, value) == false )
    return -1; // error code.
  else
    return 1;
}

/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoEncoderRead( int* flags, uint8_t* data )
{
  static const int num_bytes_encoder_transmission = 4;
  // construct array to send to Arduino:
  uint8_t write_packet[2];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = flags[0];
  
  // send the data:
  if(! serial_port_->Write_Bytes( 2, write_packet ))
  {
    ROS_ERROR("ArduinoInterface::arduinoEncoderRead(): Could not send data to Arduino");
    return -1;
  }
   
  usleep( 5000 );    
   
  // Wait for data to arrive from hardware:
  //Timeout Check Routine:   
  bool error = waitOnBytes( num_bytes_encoder_transmission );
  if( error == false )
  {
    ROS_ERROR("ArduinoInterface::arduinoEncoderRead(): Did not receive data back.");
    return -1; // error code.
  } 
  // Read value off the serial line:
  if( serial_port_->Read_Bytes( num_bytes_encoder_transmission, data ) == false )
    return -1; // error code.
  else
    return num_bytes_encoder_transmission;
}

/**********************************************************************/
/**********************************************************************/
ssize_t ArduinoInterface::arduinoEncoderWrite( int* flags, uint8_t* data )
{  
  static const int num_bytes_encoder_transmission = 4;
  // construct array to send to Arduino:
  uint8_t write_packet[6];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = flags[0];  // command byte containing object_id in upper 4 bits and command in lower 4 bits
  
  switch( flags[0] & 0x0F) // read lower 4 bits to decrypt command
  {
    case CREATE:  // creates new object on arduino
    {
      /* 
	     * the following block is Arduino Uno specific
	     */
	    //check for correct input
      switch(flags[1])  // first encoder pin
      {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13: break;
        default:
        {
	        ROS_ERROR("The selected Pin number is not available for Encoder");
	        ROS_ERROR("Select Pins 0 through 13 instead");
	        return -1;
	      }
      }
	    switch(flags[2])  // second encoder pin
      {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13: break;
        default:
        {
	        ROS_ERROR("The selected Pin number is not available for Encoder");
	        ROS_ERROR("Select Pins 0 through 13 instead");
	        return -1;
	      }
      } 
      
      write_packet[2] = flags[1];  // encoder pin 1
      write_packet[3] = flags[2];  // encoder pin 2
      // send the data:
      if(! serial_port_->Write_Bytes( 4, write_packet ))
      {
        ROS_ERROR("ArduinoInterface::arduinoEncoderWrite(): Could not send data to Arduino");
        return -1;
      }
    }break;
    
    
    case DESTROY: // destroys object on hardware device
    {
      // send the data:
      if(! serial_port_->Write_Bytes( 2, write_packet ))
      {
        ROS_ERROR("ArduinoInterface::arduinoEncoderWrite(): Could not send data to Arduino");
        return -1;
      }
    }break;
    
    
    case SET_POSITION:  // sets new position for encoder object
    {
      write_packet[2] = data[0];  // position MSB
      write_packet[3] = data[1];
      write_packet[4] = data[2];
      write_packet[5] = data[3];  // position LSB
      // send the data:
      if(! serial_port_->Write_Bytes( 6, write_packet ))
      {
        ROS_ERROR("ArduinoInterface::arduinoEncoderWrite(): Could not send data to Arduino");
        return -1;
      }
    }break;
  }
  
  //Wait for verification:
  if(!( waitOnBytes( 1 )))
    return -1;
  uint8_t verification = serial_port_->Read();
   
  if( verification != SUCCESS )
  {
    ROS_ERROR("No SUCCESS response to GPIO write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }
  return num_bytes_encoder_transmission;
}

ssize_t ArduinoInterface::arduinoAdcRead( uint8_t pin, uint8_t* data )
{
/* 
	 * the following block is Arduino Uno specific
	 */
  switch (pin)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: break;
    default:
    {
	    ROS_ERROR("The selected Pin number is not available for ADC");
	    ROS_ERROR("Select Pins 0 through 5 instead");
	    return -1;
	  }
  }
  
  // construct array to send to Arduino:
  uint8_t write_packet[2];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
	write_packet[1] = pin;
   
  // send the data:
  if(! serial_port_->Write_Bytes( 2, write_packet ))
  {
    ROS_ERROR("ArduinoInterface::arduinoAdcRead(): Could not send data to Arduino");
    return -1;
  }
   
  usleep( 5000 );    
   
  // Wait for data to arrive from sensor:
  //Timeout Check Routine:   
  bool error = waitOnBytes( 2 );
  if( error == false )
  {
    ROS_ERROR("ArduinoInterface::arduinoAdcRead(): Read broke: did not receive data back.");
    return -1; // error code.
  } 
  // Read value off the serial line:
  uint16_t value = serial_port_->Read(); // writes the MSB byte of the received value
  value = value << 8; // shifts the MSB to the proper position
  value |= serial_port_->Read(); // writes the LSB

  /* the following code would allow returning the voltage as a float but is most likely not very portable because of the reinterpret cast
  
  // voltage[V] = value(0..1023) * reference_voltage_in_mV / 1000 / max_value
  float voltage = (float)( (double)value * (double)_reference_voltage / 1000.0 / (double)MAX_ADC_VALUE );
  uint32_t transport_helper = *reinterpret_cast<uint32_t *>( &voltage ); // interprets float into uint32_t for transmission
  */
  
  // converts the read value into micro volts
  // uint64_t initialization only to not loose accuracy during calculation
  uint64_t voltage = (uint64_t)( (uint64_t)value * (uint64_t)_reference_voltage * 1000 ) / (uint64_t)MAX_ADC_VALUE;
  
  // write expects an uint_8 array, chopping voltage to four uint8_t MSB first
  // maximum voltage is 5000000 micro volts which fits into four byte 
  uint32_t temp;
  temp = (voltage & (0xFF << 24)) >> 24;
  data[0] = (uint8_t)temp;
  temp = (voltage & (0xFF << 16)) >> 16;
  data[1] = (uint8_t)temp;
  temp = (voltage & (0xFF << 8)) >> 8;
  data[2] = (uint8_t)temp;
  temp = (voltage & (0xFF << 0));
  data[3] = (uint8_t)temp;
  
  return 4; // data array filled with 4 bytes
}

ssize_t ArduinoInterface::arduinoAdcWrite( uint8_t* voltage )
{
  uint32_t temp[4];
  temp[0] = voltage[0];
  temp[1] = voltage[1];
  temp[2] = voltage[2];
  temp[3] = voltage[3];
  _reference_voltage  = temp[0] << 24;
  _reference_voltage |= temp[1] << 16;
  _reference_voltage |= temp[2] << 8;
  _reference_voltage |= temp[3] << 0;
  
  /* 
	 * the following block is Arduino Uno specific
	 */
  switch (_reference_voltage)
  {
  case 0:
  case 1:  
  case 1100:
  case 2560:
  case 5000:
    break;
  default:
    ROS_ERROR("The selected reference voltage is not available for ADC.");
    ROS_ERROR("Select 0, 1, 1100, 2560, or 5000 instead.");
    return -1;
  }
  // construct array to send to Arduino:
  uint8_t write_packet[3];
  // load it with setup parameters and data:
  write_packet[0] = data_packet_;
  write_packet[1] = voltage[2]; // transmitting only the two LSB is enough because Arduino has maximum reference value = 5000
  write_packet[2] = voltage[3];
	
  // send the data:
  serial_port_->Write_Bytes( 3, write_packet );
  usleep( 5000 );    //TODO remove?
  //Wait for verification:
  if(!( waitOnBytes( 1 )))
    return -1;
   
  uint8_t verification = serial_port_->Read();
   
  if( verification != SUCCESS )
  {
    ROS_INFO("No SUCCESS response to GPIO write prompt. Instead, 0x%x", verification);
    return -1; // error code.
  }
  return 4; // 4 bytes have been processed
}
