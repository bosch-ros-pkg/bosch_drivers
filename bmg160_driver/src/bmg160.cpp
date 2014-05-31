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

#include "bmg160_driver/bmg160.hpp"
#include "bmg160_driver/bmg160_parameters.hpp"

/**********************************************************************/
// Constructor
/**********************************************************************/
BMG160::BMG160( bosch_hardware_interface* hw ) :
  sensor_driver( hw ),
  BMG160Parameters(),
  TempSlope_( 0.5 )
{
}


/**********************************************************************/
// Destructor
/**********************************************************************/
BMG160::~BMG160() 
{
}


/**********************************************************************/
// Initialize sensor and hardware on requested parameters:
/**********************************************************************/
bool BMG160::initialize()
{  
  // Initialize the hardware interface with the selected parameters.
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("BMG160::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  if( this->softReset() == false )
    return false;

  usleep( 3000 ); // mandatory wait for sensor startup

  // change accel_range to the range set in the parameters class:     
  ROS_INFO( "BMG160::initialize(): adjusting Gyroscope range." );
  if( this->changeRange() == false )
    return false;
  ROS_INFO( "BMG160::initialize(): Gyroscope range adjusted." );
  // filter data at requested bandwidth, if requested in parameters:
  ROS_INFO( "BMG160::initialize(): Applying Filter parameter." );
  if( this->filterData( gyro_is_filtered_ ) == false )
    return false;
  ROS_INFO( "BMG160::initialize(): Filter parameter applied." );
  // change accelerometer bandwidth to the bandwidth set in the parameters:
  ROS_INFO( "BMG160::initialize(): Adjusting Gyroscope Bandwidth." );
  if( this->changeBandwidth() == false )
    return false;
  ROS_INFO( "BMG160::initialize(): Gyroscope Bandwidth adjusted." );
  
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMG160::softReset()
{ 
  if( this->writeToReg( ADDRESS_SOFTRESET, SOFTRESET_CMD ) == false )
  {
    ROS_ERROR( "BMG160::SoftReset(): failed." );
    return false;
  }
  usleep( 2000 ); // mandatory delay. See datasheet page
  
  return true;
}


/**********************************************************************/
/*INPUT: none directly. (indirectly) accel_range_, which was set 
 *        in the parameters.
 *OUTPUT: bool              -- whether or not it was successful.
 * This method performs two tasks:
 *    1. changes the accel_range to the accel_range set in the parameters.
 *    2. changes the class's accel_sensitivity, so that the output
 *      values will be multiplied correctly to produce correct values. 
 */
/**********************************************************************/
bool BMG160::changeRange()
{
  uint8_t local_range;
  // Read register with range settings for a local copy:
  if( this->readReg( ADDRESS_RANGE, &local_range ) == false )
    return false;
    
  // #ifdef DEBUG
  ROS_INFO( "Range bits before: %d.  Default: %d", local_range & 0x07, 0 ); // Defaults on page 30
  // #endif
    
  // add our command to change range:
  local_range &= ~(0x07); // clear old range value. Mask: b11111000
  local_range |= range_; // insert new range value.
  
  // write the adjusted register value back to the sensor's register:
  if( this->writeToReg( ADDRESS_RANGE, local_range ) == false )
    return false;
    
  // read back that register to make sure that changes worked:
  if( this->readReg( ADDRESS_RANGE, &local_range ) == false )
    return false;
    
  // Compare register values to what we expect:
  uint8_t expected_reg = range_;
  uint8_t actual_reg  = local_range & 0x07;
  
  // #ifdef DEBUG
  ROS_INFO( "Range bits after:  %d.  Expected: %d", actual_reg, expected_reg );
  // #endif
    
  if( expected_reg != actual_reg )
    return false;
    
  // if everything works:
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMG160::filterData( bool request )
{
  uint8_t local_bandwidth;
  // Read register with accel range settings for a local copy:
  if( this->readReg( ADDRESS_FILTER, &local_bandwidth ) == false )
    return false;
    
  // #ifdef DEBUG
  ROS_INFO( "filter bits before: %d.  Default: %d", ( local_bandwidth >> DATA_HIGH_BW ) & 0x01, 0 ); // Defaults from page 30.
  // #endif 

  // add our command to apply filter: 
  if( request == true )
    local_bandwidth &= ~(1 << DATA_HIGH_BW);  // filtered: write 0
  else
    local_bandwidth |= (1 << DATA_HIGH_BW);   // unfiltered: write 1
    
  // write the adjusted register value back to the sensor's register:
  if( this->writeToReg( ADDRESS_FILTER, local_bandwidth ) == false )
    return false;
    
  // read back that register to make sure that changes worked:
  if( this->readReg( ADDRESS_FILTER, &local_bandwidth ) == false )
    return false;
    
  // Compare register values to what we expect:
  uint8_t expected_reg = !gyro_is_filtered_;
  uint8_t actual_reg  = ( local_bandwidth >> DATA_HIGH_BW ) & 0x01;
  
  // #ifdef DEBUG                                                       
  ROS_INFO( "filter bits after:  %d.  Expected: %d", actual_reg, expected_reg );
  // #endif
    
  if( expected_reg != actual_reg )
    return false;
    
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMG160::changeBandwidth()
{
  uint8_t local_bw_reg;
  // read reg for a local copy:
  if( this->readReg( ADDRESS_BANDWIDTH, &local_bw_reg ) == false )
    return false;
  
  ROS_INFO( "bandwidth setting before: %d.  Default:  %d", local_bw_reg & 0x0F, 0 ); // Defaults from page 50.
  
  // apply new bandwidth value:
  local_bw_reg &= ~(0x0F); //Mask: 1111000
  local_bw_reg |= bw_reg_;
  // write back reg with new bandwidth value:
  if( this->writeToReg( ADDRESS_BANDWIDTH, local_bw_reg ) == false )
    return false;
  // verify that we wrote back the correct value:
  uint8_t actual_reg;
  if( this->readReg( ADDRESS_BANDWIDTH, &actual_reg ) == false )
    return false;
  
  uint8_t actual_bw = actual_reg & 0x0F; // mask off unrelated bits.
  uint8_t expected_bw = bw_reg_;
  
  if( expected_bw != actual_bw )
  {
    ROS_ERROR( "bandwidth setting after: %d.  Expected: %d", actual_bw, bw_reg_ );
    ROS_ERROR( "BMG160::changeBandwidth(): failed." );
    return false;
  }
  else
    ROS_INFO( "bandwidth setting after: %d.  Expected: %d", actual_bw, bw_reg_ );

  return true;
}

  
/**********************************************************************/
/**********************************************************************/
void BMG160::SimpleCalibration()
{
  uint8_t local_enable_cal_reg; 
  uint8_t calibration_done;
  uint8_t data_is_filtered;
  // check whether or not gyro is filtered: (read register directly)
  if( this->readReg( ADDRESS_FILTER, &data_is_filtered ) == false )
  {
    std::cout<< "Calibration Error occured" << std::endl;
    return;
  }
  data_is_filtered = !(bool)((data_is_filtered & (1 << DATA_HIGH_BW) >> DATA_HIGH_BW));
  // make sure this is what the class thinks:
  if( gyro_is_filtered_ != (bool)data_is_filtered )
  {
    ROS_ERROR("Calibration Error occured.");
    return;
  }
    
  // tell sensor which version of calibration it should perform:
  // the filtered version or unfiltered version, depending on user's
  // configuration:
  uint8_t local_filtered_reg;
  if( this->readReg( ADDRESS_FILTERED_CAL_FAST, &local_filtered_reg ) == false )
  {
    ROS_ERROR("Calibration Error occured.");
    return;
  }
  // Apply our changes:
  if( (bool)data_is_filtered )
    local_filtered_reg &= ~(1 << fast_offset_unfilt); // write 0
  else
    local_filtered_reg |= (1 << fast_offset_unfilt); // write 1
  // Write back Calibration type:
  if( this->writeToReg( ADDRESS_FILTERED_CAL_FAST, local_filtered_reg ) == false )
  {
    ROS_ERROR("Calibration Error occured.");
    return;
  }

  // Query User
  std::cout << "Place the sensor in a static position.  Press [y/n] to continue/cancel." << std::endl;
  std::string reply;
  std::getline( std::cin, reply );
  if( reply != "Y" && reply != "y" )
  {
    ROS_WARN("Calibration cancelled by user.");
    return;
  }   
  
  // Begin calibration on each axis simultaneously:
  std::cout << "Beginning Fast Calibration routine:" << std::endl << std::endl;
 
  if( this->readReg( ADDRESS_ENABLE_FAST_CAL, &local_enable_cal_reg ) == false )
  {
    std::cout<< "Calibration Error occured" << std::endl;
    return;
  }
  // set bits for all three axes:
  local_enable_cal_reg |= 0x07;  
  // add in fast_offset_en bit:
  local_enable_cal_reg |= (1 << fast_offset_en);
    
  if( this->writeToReg( ADDRESS_ENABLE_FAST_CAL, local_enable_cal_reg ) == false )
  {
    std::cout<< "Calibration Error occured" << std::endl;
    return;
  }
  std::cout << "Calibrating..." << std::endl;
    
  // continue reading fast_offset_en bit until it is reset to zero:
  do
  {
    this->readReg( ADDRESS_ENABLE_FAST_CAL, &calibration_done );
    calibration_done &= (1 << fast_offset_en);
    calibration_done = calibration_done >> fast_offset_en;
  }
  while( calibration_done == 1);
  
  std::cout << "GYROSCOPE CALIBRATION COMPLETE" << std::endl;
  
  // Print final register settings (filtered and unfiltered):
  this->printOffsets(); 
}


/**********************************************************************/
// gets raw data and updates GyroX_, GyroY_, GyroZ_, and Temperature_
/**********************************************************************/
bool BMG160::takeMeasurement()
{
  uint8_t Data[7];

  if( this->readSensorData( ADDRESS_GYRO_X_LSB, Data, 7 ) < 0 )
  {
    ROS_ERROR("BMG160::takeMeasurement(): Error reading from hardware interface!");
    return false;
  }
    
  uint16_t tempx = Data[1];
  uint16_t tempy = Data[3];
  uint16_t tempz = Data[5];
  GyroX_ = this->getSensitivity() * (int16_t)( (tempx << 8) | Data[0] );
  GyroY_ = this->getSensitivity() * (int16_t)( (tempy << 8) | Data[2] );
  GyroZ_ = this->getSensitivity() * (int16_t)( (tempz << 8) | Data[4] );
  
  Temperature_ = (TempSlope_ *(int8_t)Data[6]) + 24;            
              
  return true;
}


/**********************************************************************/
// gets raw data and updates : GyroX_
/**********************************************************************/
double BMG160::getGyroX()
{
  uint8_t Data[2];

  if( this->readSensorData( ADDRESS_GYRO_X_LSB, Data, 2) < 0 )
  {
    ROS_ERROR("BMG160::getGyroX(): Error reading from hardware interface!");
    return false;
  }
  
  uint16_t temp = Data[1];
  GyroX_ = this->getSensitivity() * (int16_t)( (temp << 8) | Data[0] );
  return GyroX_;
}
  
 
/**********************************************************************/
// gets raw data and updates : GyroY_
/**********************************************************************/
double BMG160::getGyroY()
{
  uint8_t Data[2];
  
  if( this->readSensorData( ADDRESS_GYRO_Y_LSB, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMG160::getGyroY(): Error reading from hardware interface!");
    return false;
  }
  
  uint16_t temp = Data[1];
  GyroY_ = this->getSensitivity() * (int16_t)( (temp << 8) | Data[0] );
  return GyroY_;
}


/**********************************************************************/
// gets raw data and updates : GyroZ_
/**********************************************************************/
double BMG160::getGyroZ()
{
  uint8_t Data[2];
  
  if( this->readSensorData( ADDRESS_GYRO_Z_LSB, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMG160::getGyroZ(): Error reading from hardware interface!");
    return false;
  }

  uint16_t temp = Data[1];
  GyroZ_ = this->getSensitivity() * (int16_t)( (temp << 8) | Data[0] );
  return GyroZ_;
}


/**********************************************************************/
// gets raw data and updates : Temperature_
/**********************************************************************/
double BMG160::getTemperature()
{
  uint8_t Data[1];
  
  if( this->readSensorData( ADDRESS_TEMPERATURE, Data, 1 ) < 0 )
  {
    ROS_ERROR("BMG160::getTemperature(): Error reading from hardware interface!");
    return false;
  }

  Temperature_ = ( TempSlope_ * (int8_t)Data[0] ) + 24;         
  return Temperature_;
}


/**********************************************************************/
/**********************************************************************/
uint8_t BMG160::getDeviceAddress()
{
  // depends on the protocol:
  switch( this->getProtocol() )
  {
  case I2C:
    // depends on hardware configuration:
    if( this-> getSlaveAddressBit() == false )
      return SLAVE_ADDRESS0;  
    else 
      return SLAVE_ADDRESS1;
    break;
  case SPI:
    return this->getPin();
  default:
    ROS_ERROR("BMG160::getAccelAddress(): invalid protocol.");
    return 0;
  }
}


/**********************************************************************/
// reads a register and returns its value to the second argument
/**********************************************************************/
bool BMG160::readReg( uint8_t reg, uint8_t* value )
{
  // Reading depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->read( this->getDeviceAddress(), I2C, this->getFrequency(), this->getFlags(), reg, value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMG160::readReg(): Error reading register via I2C!");
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG.
    if( hardware_->read( this->getDeviceAddress(), SPI, this->getFrequency(), this->getFlags(), (1 << SPI_READ_FLAG) | reg, value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMG160::readReg(): Error reading register via SPI!");
      return false;
    } 
    break;
  default:
    ROS_ERROR("BMG160::readReg(...): invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMG160::writeToReg( uint8_t reg, uint8_t value )
{
  // Technically, writing depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->write( this->getDeviceAddress(), I2C, this->getFrequency(), this->getFlags(), (uint8_t)reg, (uint8_t*)&value, 1 ) < 0 )
    {
      ROS_ERROR("BMG160::writeToReg(): Error writing to register via I2C!");
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_WRITE_FLAG, although, technically it's already there, since it's zero.
    if( hardware_->write( this->getDeviceAddress(), SPI, this->getFrequency(), this->getFlags(), (uint8_t) (~(1 << SPI_WRITE_FLAG)&reg), (uint8_t*)&value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMG160::writeToReg(): Error writing to register via SPI!");
      return false;
    } 
    break;
  default:
    // shouldn't happen:
    ROS_ERROR("BMG160::writeToReg(...): invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMG160::writeToRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected )
{
  this->writeToReg( reg, value );
  
  uint8_t actual;
  
  this->readReg( reg, &actual );
  
  if( expected != actual )
  {
    ROS_ERROR("BMG160::writeToRegAndVerify(...): Register: %d  actual: %d  expected: %d", reg, actual, expected); 
    return false;
  }
  
  // if it all works:
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMG160::readSensorData( uint8_t reg, uint8_t* data_array, uint8_t num_bytes )
{
  // Reading depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->read( this->getDeviceAddress(), I2C, this->getFrequency(), this->getFlags(), reg, data_array, num_bytes ) < 0 ) 
    {
      ROS_ERROR("BMG160::readSensorData(): Error reading register via I2C!");
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG.
    if( hardware_->read( this->getDeviceAddress(), SPI, this->getFrequency(), this->getFlags(), (1 << SPI_READ_FLAG) | reg, data_array, num_bytes ) < 0 ) 
    {
      ROS_ERROR("BMG160::readSensorData(): Error reading register via SPI!");
      return false;
    } 
    break;
  default:
    ROS_ERROR("BMG160::readSensorData(...): invalid protocol.");
    return false;
  }
  return true;  
}


/**********************************************************************/
/**********************************************************************/
void BMG160::computeOffsets( uint8_t* raw_offset_data, int16_t* clean_offset_data )
{
  // format offset x:
  clean_offset_data[0] = raw_offset_data[1] << 8;
  clean_offset_data[0] |= (raw_offset_data[0] & 0xC0); // mask: 0b11000000
  clean_offset_data[0] |= ((uint8_t)(raw_offset_data[4] & 0x0C) << 2); // mask: 0b00001100
  clean_offset_data[0] = clean_offset_data[0] >> 4; // sign-extend 12bit to 16bit
  // format offset y:
  clean_offset_data[1] = raw_offset_data[2] << 8;
  clean_offset_data[1] |= ((uint8_t)(raw_offset_data[0] & 0x38) << 2);
  clean_offset_data[1] |= ((uint8_t)(raw_offset_data[4] & 0x02) << 3);
  clean_offset_data[1] = clean_offset_data[1] >> 4; // sign-extend 12bit to 16bit
  // format offset z:
  clean_offset_data[2] = raw_offset_data[3] << 8;
  clean_offset_data[2] |= ((uint8_t)raw_offset_data[0] << 5);
  clean_offset_data[2] |= ((uint8_t)(raw_offset_data[4] & 0x01) << 4);
  clean_offset_data[2] = clean_offset_data[2] >> 4; // sign-extend 12bit to 16bit
}


/**********************************************************************/
// prints computed offset values
/**********************************************************************/
bool BMG160::printOffsets()
{
  std::string offset_reg_names[3] = { "offset_x", "offset_y", "offset_z" };
  uint8_t raw_offset_data[5];
  int16_t clean_offset_data[3];
  
  if( this->readSensorData( ADDRESS_OFFSETS, raw_offset_data, 5 ) == false )
    return false;
    
  this->computeOffsets( raw_offset_data, clean_offset_data );
  
  // print:
  for( uint8_t i = 0; i < 3; i++ )
  {
    std::cout << offset_reg_names[i] << ": " << (int)clean_offset_data[i] << std::endl;
  }
  return true;
}
