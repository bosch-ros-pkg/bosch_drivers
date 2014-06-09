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

#include <Wire.h>
#include <SPI.h>
#include "Encoder.h"

#include <arduino_constants.hpp>

/* Important notes:
    
*** IMPORTANT ***
for i2c interfaces you MUST DISABLE internal pullups
on the SDA and SCL lines. This must be performed in 
the twi.c file, located in 
    
libraries/Wire/utility 
    
Simply comment out the following two lines:
       
digitalWrite(SDA, 1);        
digitalWrite(SCL, 1);
  
Furthermore, you must add your own external pullup
resistors on SDA and SCL lines.  Choose values between 
1 and 10 [kOhm].
    
***           ***
   
This sketch assumes that you do not read or write
more than 32 bytes per transaction since data is
transferred directly out of its respective data
buffer.
    
Note:
i2c data buffer      32 bytes
Serial data buffer   64 bytes
    
Note: SPI transfers can transfer a single byte if the 
number of bytes is zero.  In this case, the SPI will 
just transfer the register argument to the slave as a 
single byte.
*/


// Serial variables:
uint8_t incoming_byte = 0;
uint8_t data_buffer[64]; // to store data to write
uint8_t protocol;
uint8_t read_or_write;
uint8_t frequency;

// SPI variables:
uint8_t bit_order;
uint8_t mode;
uint8_t spi_slave_select;
uint8_t flags_;

// main loop variables
bool state_ = false;

// encoder variables
Encoder* encoders[16];

// Function prototypes

bool i2c_read_routine();
bool i2c_write_routine();
bool change_i2c_frequency( uint8_t new_freq );

bool spi_read_routine();
bool spi_write_routine();
void spi_slave_low( uint8_t spi_slave );
void spi_slave_high( uint8_t spi_slave );
bool pwm_write_routine();
bool gpio_read_routine();
bool gpio_write_routine();
bool encoder_write();
bool encoder_read();
bool adc_read();
bool adc_write();

// initial setup
void setup()
{
  Serial.begin( bosch_drivers_common::DEFAULT_BAUD_RATE );
  pinMode( 13, OUTPUT );
  Serial.flush();
}

// main loop, a finite-state machine (FSM)
void loop()
{
  while( Serial.available() < 1 )
    ; // wait for control bytes:
  incoming_byte = Serial.read();
  
  state_ = true;
  
  // Decrypte protocol with bitmasking:
  protocol = (B11111110 & incoming_byte) >> 1;
  read_or_write = B00000001 & incoming_byte;
  
  // Identify Protocol
  switch( protocol )
  {
  case bosch_drivers_common::I2C:
    Wire.begin();
    // read frequency
    while( Serial.available() < 1 )
      ; // wait for next byte
    frequency = Serial.read();
    state_ = change_i2c_frequency( frequency );
    if( read_or_write == bosch_drivers_common::READ )
      state_ = i2c_read_routine();
    else if( read_or_write == bosch_drivers_common::WRITE )
      state_ = i2c_write_routine();
    else
      state_ = false;
    break;

  case bosch_drivers_common::SPI:
    SPI.begin();
    // read frequency
    while( Serial.available() < 1 )
      ; // wait for next byte
    frequency = Serial.read();
    SPI.setClockDivider( frequency );
    if( read_or_write == bosch_drivers_common::READ )
      state_ = spi_read_routine();
    else if( read_or_write == bosch_drivers_common::WRITE )
      state_ = spi_write_routine();
    else
      state_ = false;
    break;

  case bosch_drivers_common::PWM:
    if( read_or_write == bosch_drivers_common::WRITE )
      state_ = pwm_write_routine();
    else
      state_ = false;
    break;
    
  case bosch_drivers_common::GPIO:
    if( read_or_write == bosch_drivers_common::READ )
      state_ = gpio_read_routine();
    else if( read_or_write == bosch_drivers_common::WRITE )
      state_ = gpio_write_routine();
    else
      state_ = false;
    break;

  case bosch_drivers_common::ENCODER:
    if( read_or_write == bosch_drivers_common::READ )
      state_ = encoder_read();
    else if( read_or_write == bosch_drivers_common::WRITE )
      state_ = encoder_write();
    else
      state_ = false;
    break;
    
    case bosch_drivers_common::ADCONVERTER:
    if( read_or_write == bosch_drivers_common::READ )
      state_ = adc_read();
    else if( read_or_write == bosch_drivers_common::WRITE )
      state_ = adc_write();
    else
      state_ = false;
    break;
    
  default:
    // an error occured
    state_ = false;
  } 
  // Error Routine: Wait forever with LED ON.
  if( state_ != true )
  {
    digitalWrite( 13, HIGH );
    while( true )
      ;
  }
}


/*------------Function Definitions----------------------*/

/*** I2C routines ***/

/* i2c_read_routine()
 * output: returns false if the sensor does not reply.
 */
bool i2c_read_routine()
{
  // Wait for 3 bytes:
  while( Serial.available() < 3 )
    ; 

  // who are we reading from:
  uint8_t slave_address = Serial.read();
  // what register are we reading
  uint8_t slave_register = Serial.read();
  // how many bytes to request from the slave:
  uint8_t num_bytes = Serial.read();
  
  // Send back verification:
  Serial.write( bosch_drivers_common::VERIFY );
    
  Wire.beginTransmission( slave_address );
  Wire.write( slave_register );
  Wire.endTransmission();  // send data
    
  // Request data:
  Wire.requestFrom( slave_address, num_bytes ); 
  // Unspeedy Version:
  for( int i = 0; i < num_bytes; i++ )
  {
    while( Wire.available() < 1 )
      ; // wait for next byte.
    data_buffer[i] = Wire.read();
  }
  // End: new version

  // Write back data byte-by-byte:
  for( int j = 0;j < num_bytes; j++ )
  {
    Serial.write( data_buffer[j] );
  }
  /*    BEGIN: UNTESTED SPEEDY VERSION   
  // Wait for requested data:
  while (Wire.available() < data_size); 
  // Write back data directly out of the I2C buffer:
  for (int i=0;i<data_size;i++)
  {
  Serial.write( Wire.read() );
  }
  END:    UNTESTED SPEEDY VERSION
  */ 
  return true;
}

bool i2c_write_routine()
{
  // wait for 3 bytes:
  while( Serial.available() < 3 )
    ; // do nothing.
  // who we are writing to:
  uint8_t slave_address = Serial.read();
  // what register we are writing to:
  uint8_t slave_register = Serial.read();
  // how many bytes we will write:
  uint8_t num_bytes = Serial.read();
  // get bytes to write from the serial line:

  Serial.write( bosch_drivers_common::VERIFY );
  
  // Wait for all data to arrive:
  while( Serial.available() < num_bytes )
    ;
  
  Wire.beginTransmission( slave_address );
  Wire.write(slave_register);
  // write our data:
  for( int i = 0; i < num_bytes; i++ )
  {
    // write data directly out of the serial buffer:
    Wire.write( Serial.read() )
      ;
  }
  Wire.endTransmission();
    
  //Send back an acknowledgement
  Serial.write( bosch_drivers_common::SUCCESS );  
  
  return true;
}


/* change_i2c_frequency( long new_freq)
 *   input:   requested new frequency:
 *   output:  true, if successful; false, otherwise.
 */
bool change_i2c_frequency( uint8_t new_freq )
{  // See datasheet on changing the i2c frequency.
  // Change Prescaler in TWDR to the value: 1.
  TWSR &= B11111100;
  // Change TWBR value
  switch( new_freq )
  {
  case bosch_drivers_common::FREQ_STANDARD: // 100 [kHz]  
    TWBR = 72;
    break; 
  case bosch_drivers_common::FREQ_FAST: //  400 [kHz]
    TWBR = 12;
    break;
  default:
    return false;
  }
  return true;
}



/* SPI ROUTINES */

bool spi_read_routine()
{
  // wait for 3 bytes:
  while( Serial.available() < 3 )
    ;

  // who we are writing to:
  uint8_t flags = Serial.read();
  // what register we are writing to:
  uint8_t slave_register = Serial.read();
  // how many bytes we will write:
  uint8_t num_bytes = Serial.read();
  
  pinMode( spi_slave_select, OUTPUT );
  
  //  Decrypt flags:
  bit_order = (flags>>2) & B00000001;
  mode = flags & B00000011;
  spi_slave_select = (flags >> 4) & B00001111;
  
  SPI.setBitOrder( bit_order );
  
  switch( mode )
  {
  case bosch_drivers_common::SPI_MODE_0:
    SPI.setDataMode( SPI_MODE0 );
    break;
  case bosch_drivers_common::SPI_MODE_1:
    SPI.setDataMode( SPI_MODE1 );
    break;
  case bosch_drivers_common::SPI_MODE_2:
    SPI.setDataMode( SPI_MODE2 );
    break;
  case bosch_drivers_common::SPI_MODE_3:
    SPI.setDataMode( SPI_MODE3 );
    break;
  default:
    return false;
  }
  
  // Send the PC a data verification code:
  Serial.write( bosch_drivers_common::VERIFY );

  // Do not toggle chip_select pin if spi_slave is a NULL_DEVICE:
  if( spi_slave_select != bosch_drivers_common::NULL_DEVICE )
  {  // activate chip-select:
    spi_slave_low( spi_slave_select );
  }

  
  // transfer the address register:
  SPI.transfer( slave_register );
  
  for( int i = 0; i < num_bytes; i++ )
  {
    data_buffer[i] = SPI.transfer( 0xFF );
  }
  
  // Do not toggle chip_select pin if spi_slave is a NULL_DEVICE:
  if( spi_slave_select != bosch_drivers_common::NULL_DEVICE )
  {// deactivate chip-select
    spi_slave_high( spi_slave_select );
  }
  
  // Write back data byte-by-byte:
  for( int j = 0; j < num_bytes; j++ )
  {
    Serial.write( data_buffer[j] ); 
  }
  return true;
}

bool spi_write_routine()
{
  // wait for 3 bytes:
  while( Serial.available() < 3 )
    ;

  // flags containing bit_order, mode, pin:
  uint8_t flags = Serial.read();
  // what register we are writing to:
  uint8_t slave_register = Serial.read();
  // how many bytes we will write:
  uint8_t num_bytes = Serial.read();
  // get bytes to write from the serial line:
  
  //  Decrypt flags:
  bit_order = ( flags >> 2 ) & B00000001;
  mode = flags & B00000011;
  spi_slave_select = (flags >> 4) & B00001111;
  
  SPI.setBitOrder( bit_order );
  
  switch( mode )
  {
  case bosch_drivers_common::SPI_MODE_0:
    SPI.setDataMode( SPI_MODE0 );
    break;
  case bosch_drivers_common::SPI_MODE_1:
    SPI.setDataMode( SPI_MODE1 );
    break;
  case bosch_drivers_common::SPI_MODE_2:
    SPI.setDataMode( SPI_MODE2 );
    break;
  case bosch_drivers_common::SPI_MODE_3:
    SPI.setDataMode( SPI_MODE3 );
    break;
  default:
    return false;
  }  
  
  //CHIP SELECT LOW... or do not toggle chip_select pin if spi_slave is a NULL_DEVICE:
  if( spi_slave_select != bosch_drivers_common::NULL_DEVICE )
  { // Toggle Device otherwise:
    // activate chip-select:
    spi_slave_low( spi_slave_select );
  }
    
  while( Serial.available() < num_bytes )
    ; // wait for data
    
  // Send verify code:
  Serial.write( bosch_drivers_common::VERIFY );
  
  /*********************************/ 
  // BEGIN CASE: zero-byte transfer:
  // Transfer only register value.  This is equivalent to an SPI transfer of a single byte.
  if( num_bytes == 0 )
  {
    SPI.transfer( slave_register );    
    
    // Do not toggle chip_select pin if spi_slave is a NULL_DEVICE:
    if( spi_slave_select != bosch_drivers_common::NULL_DEVICE )
    { // Toggle Device Otherwise:
      // deactivate chip-select
      spi_slave_high( spi_slave_select );
    }
    return true;
  }
  // END CASE: zero-byte transfer.
  /*******************************/
 

  // Otherwise, transfer byte-by-byte while incrementing buffer:
  SPI.transfer( slave_register );
  // transfer data directly from the serial buffer:
  // assume data is sequential in memory!!!
  for( int i = 0; i < num_bytes; i++ )
  {
    SPI.transfer( Serial.read() );
  }

  //CHIP SELECT HIGH... or do not toggle chip_select pin if spi_slave is a NULL_DEVICE:
  if( spi_slave_select != bosch_drivers_common::NULL_DEVICE )
  { // Toggle Device Otherwise:
    // deactivate chip-select
    spi_slave_high( spi_slave_select );
  }
  return true; 
}

void spi_slave_low( uint8_t spi_slave )
{
  if( spi_slave > 1 && spi_slave < 8 )
  {
    DDRD |= 1 << spi_slave; // configure pin as output.
    PORTD &= 0xFF & ~(1 << spi_slave); // set pin low.
  }
  else
  {
    DDRB |= 1 << spi_slave; // configure pin as output.
    PORTB &= 0xFF & ~(1 << (spi_slave - 8));
  }
}

void spi_slave_high( uint8_t spi_slave )
{
  if( spi_slave > 1 && spi_slave < 8 )
  {
    PORTD |= 1 << spi_slave;
  }
  else
  { 
    PORTB |= 1 << (spi_slave - 8);
  }
}

bool pwm_write_routine()
{
  // wait for 2 bytes
  while( Serial.available() < 2 )
    ;

  uint8_t pwm_channel = Serial.read();
  uint8_t pwm_duty_cycle = Serial.read();
  analogWrite( pwm_channel, pwm_duty_cycle );
  // send back a success
  Serial.write( bosch_drivers_common::SUCCESS );
  return true;
}

bool gpio_write_routine()
{
  // wait for 2 bytes
  while( Serial.available() < 2 )
    ;

  uint8_t gpio_pin = Serial.read();
  uint8_t gpio_value = Serial.read();
  // set GPIO pin to output
  pinMode( gpio_pin, OUTPUT );
  
  // set GPIO pin to desired value
  if( gpio_value == 1 )
  {
    digitalWrite(gpio_pin, HIGH);
  }
  else if( gpio_value == 0 )
  {
    digitalWrite(gpio_pin, LOW);
  }    
  else
  {
    return false;
  }
  // send back a success
  Serial.write( bosch_drivers_common::SUCCESS );
  return true;
}

bool gpio_read_routine()
{
  // wait for 2 bytes
  while( Serial.available() < 2 )
    ;

  uint8_t flags = Serial.read();
  uint8_t gpio_pin = Serial.read();
  switch( flags )
  {
  case bosch_drivers_common::FLOATING: 
    pinMode( gpio_pin, INPUT );
    break;
  case bosch_drivers_common::PULLUP:
    pinMode( gpio_pin, INPUT_PULLUP );
    break;
  default: 
    return false;
  }
    
  // send back the read value
  Serial.write( digitalRead( gpio_pin ) );
  return true;
}

bool encoder_write()
{
  long ticks;
  // wait for 1 byte
  while( Serial.available() < 1 )
    ;

  uint8_t control_byte = Serial.read();
  switch( control_byte & 0x0F ) // read lower 4 bits to decrypt right command
  {
  case bosch_drivers_common::CREATE:  // create a new encoder object
    while( Serial.available() < 2 )
      ;  // wait for hardware pin configuration to be transmitted
    
    uint8_t encoder_pin1 = Serial.read();
    uint8_t encoder_pin2 = Serial.read();
    // create new encoder object in Encoder pointer array at the position defined by the upper 4 bits in the control byte
    encoders[(control_byte & 0xF0) >> 4] = new Encoder( encoder_pin1, encoder_pin2 );
    break;
    
  case bosch_drivers_common::DESTROY: // destroys an encoder object
    delete encoders[(control_byte & 0xF0) >> 4];
    break;
    
  case bosch_drivers_common::SET_POSITION:  // sets the encoder value
    // convert read serial data to long
    unsigned long temp[4];
    while( Serial.available() < 4 );
    temp[0] = Serial.read();
    temp[1] = Serial.read();
    temp[2] = Serial.read();
    temp[3] = Serial.read();
    ticks  = temp[0] << 24;
    ticks |= temp[1] << 16;
    ticks |= temp[2] << 8;
    ticks |= temp[3] << 0;
    
    encoders[(control_byte & 0xF0) >> 4]->write( ticks );
    break;
    
  default:
    return false;
  }
  
  // send back a success
  Serial.write( bosch_drivers_common::SUCCESS );
  
  return true;
}

bool encoder_read()
{
  // wait for 1 byte
  while( Serial.available() < 1 )
    ;

  uint8_t control_byte = Serial.read();
  
  // reads the encoder value from the encoder object at the position defined by the upper 4 bits in the control byte
  long ticks = encoders[(control_byte & 0xF0) >> 4]->read();
  // convert long variable (4 Bytes long) by bitshifting
  Serial.write( (byte)((ticks & 0xFF000000) >> 24) );
  Serial.write( (byte)((ticks & 0x00FF0000) >> 16) );
  Serial.write( (byte)((ticks & 0x0000FF00) >> 8 ) );
  Serial.write( (byte)((ticks & 0x000000FF) >> 0 ) );
  return true;
}

bool adc_read()
{
  // wait for 1 byte
  while( Serial.available() < 1 )
    ;

  uint8_t adc_pin = Serial.read();
  uint16_t adc_value;
  
  adc_value = (uint16_t)analogRead( adc_pin );

  Serial.write( (byte)((adc_value & 0xFF00) >> 8 ) );
  Serial.write( (byte)((adc_value & 0x00FF) >> 0 ) );
  return true;
}

bool adc_write()
{
  // wait for 2 bytes
  while( Serial.available() < 2 )
    ;

  uint16_t reference = Serial.read(); // writes the MSB byte of the reference voltage
  reference = reference << 8; // shifts the MSB to the proper position
  reference |= Serial.read(); // writes the LSB
  switch ( reference )
  {
  case 0:
    analogReference(EXTERNAL);
    break;
#ifdef __AVR_ATmega328P__
  case 1:
    analogReference(INTERNAL);
    break;
#endif
#ifdef __AVR_ATmega2560__
  case 1100:
    analogReference(INTERNAL1V1);
    break;
  case 2560:
    analogReference(INTERNAL2V56);
    break;
#endif
  case 5000:
    analogReference(DEFAULT);
    break;
  default:
    return false;
  }
  // send back a success
  Serial.write( bosch_drivers_common::SUCCESS );
  return true;
}
