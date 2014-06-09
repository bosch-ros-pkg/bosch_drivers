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

// For calibration routine
#include <iostream> 

// ROS headers for debugging output
#include <ros/console.h>

#include "bma180_driver/bma180.hpp"


/**********************************************************************/
// Constructor
/**********************************************************************/
BMA180::BMA180( bosch_hardware_interface* hw ) :
  sensor_driver( hw, EXTERNAL_DEVICE ),
  TempSlope_( 0.5 ),
  accel_range_( RANGE_2),
  sensitivity_( 0.00025 ),
  bandwidth_( BW_150 ),
  slave_address_bit_( 0 )
{
  sensor_parameters_->device_address = SLAVE_ADDRESS0;
  sensor_parameters_->protocol = I2C;
  sensor_parameters_->frequency = 400000;
  //byte_order( MSB_FIRST ),
  //spi_mode( SPI_MODE_3 )
  //sensor_parameters_->flags = ;

}


/**********************************************************************/
// Destructor
/**********************************************************************/
BMA180::~BMA180()
{
}


/**********************************************************************/
/**********************************************************************/
bool BMA180::setDeviceAddress( uint8_t address )
{
  sensor_parameters_->device_address = address;
  return true;
}

uint8_t BMA180::getDeviceAddress()
{
  // depends on the protocol:
  switch( sensor_parameters_->protocol )
  {
  case I2C:
    // depends on hardware configuration:
    if( slave_address_bit_ == 0 )
      return SLAVE_ADDRESS0; 
    else
      if( slave_address_bit_ == 1 )
        return SLAVE_ADDRESS1;
      else   
        ROS_ERROR( "BMA180::getDeviceAddress(): invalid I2C address" );
    break;
  case SPI:
    return sensor_parameters_->device_address;
  default:
    ROS_ERROR( "BMA180::getDeviceAddress(): sensor has no identification. Either setPin(uint8_t pin) for SPI or setSlaveAddress( 0 or 1) for I2C." );
    return 255;
  }
  ROS_ERROR( "BMA180::getDeviceAddress(): sensor has no identification. Either setPin(uint8_t pin) for SPI or setSlaveAddress( 0 or 1) for I2C." );
  return 255;
}

bool BMA180::setParameters( bosch_driver_parameters parameters )
{
  *sensor_parameters_ = parameters;
  return true;
}

bool BMA180::setFrequency( unsigned int frequency )
{
  sensor_parameters_->frequency = frequency;
  return true;
}

/**********************************************************************/
// Initialize sensor and hardware interface on user-requested parameters
/**********************************************************************/
bool BMA180::initialize()
{
  ROS_INFO( " " );
  ROS_INFO( "BMA180::initialize(): Device Address (hex): %x",this->getDeviceAddress() );
  ROS_INFO( "BMA180::initialize(): Protocol:             %d",this->getProtocol() );
  ROS_INFO( "BMA180::initialize(): Frequency:            %d",this->getFrequency() );
 
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
    return false;

  // Reset the sensor:
  if( this->softReset() == false )
    return false;
  ROS_INFO( "BMA180::initialize(): soft reset applied." );

// Disable I2C, if sensor is setup for SPI mode:
  switch( this->getProtocol() )
  {
  case I2C:
    break;
  case SPI:
    // disable i2c to prevent accidental malfunctions:
    if( this->DisableI2C() == false )
      return false;
    ROS_INFO( "BMA180::initialize(): Disabled I2C mode." );
    break;
  default: // shouldn't happen. User should know to select either SPI or I2C.
    ROS_ERROR( "BMA180::initialize(): BMA180 cannot be read with selected protocol." );
    return false;
  }
  
  // Enable EEPROM and Register Writing:
  if( this->EnableWriting() == false )
    return false;
  
  // Change accel_range on the sensor to match requested parameter range:
  if( this->setAccelerationRange( accel_range_ ) == false )
    return false;

  // Change bandwidth_ on the sensor to match requested parameter bandwidth:
  if( this->changeBandwidth() == false )
    return false;

  ROS_INFO( "BMA180 initialized." );
  sleep( .1 );

  return true;
}


/**********************************************************************/
// takeMeasurement
/**********************************************************************/
bool BMA180::takeMeasurement()
{
  uint8_t Data[7];
  if( this->readSensorData( ADDRESS_ACCLXYZ, Data, 7 ) == false )
  {
    ROS_ERROR(" BMA180::takeMeasurement(): Unable to read accelerometer data from sensor." );
    return false;
  }
             
  AccelX_ = sensitivity_ * int( ( ((int16_t)(Data[1] << 8)) | ( ((int16_t)Data[0]) ) ) >> 2 ); // [g]
  AccelY_ = sensitivity_ * int( ( ((int16_t)(Data[3] << 8)) | ( ((int16_t)Data[2]) ) ) >> 2 ); // [g]
  AccelZ_ = sensitivity_ * int( ( ((int16_t)(Data[5] << 8)) | ( ((int16_t)Data[4]) ) ) >> 2 ); // [g]
  Temperature_ = ( TempSlope_* (int8_t)Data[6] ) + 24; // [C]
  return true;
}

/**********************************************************************/

/**********************************************************************/
double BMA180::getStaticPitch()
{
  // returns the pitch based on the most recent measurements
  StaticPitch_ = atan2( AccelY_, AccelZ_ ) ;
  return StaticPitch_;
}

/**********************************************************************/

/**********************************************************************/
double BMA180::getStaticRoll()
{
  StaticRoll_ = atan2( AccelX_, AccelZ_ );
  return ( StaticRoll_ );
}


/**********************************************************************/

/**********************************************************************/
bool BMA180::getAccelData()
{
  uint8_t Data[6];
  if( this->readSensorData( ADDRESS_ACCLXYZ, Data, 6 ) == false )
  {
    ROS_ERROR( "BMA180::getAccelData(): Unable to read accelerometer data from sensor." );
    return false;
  }
  AccelX_ = this->getSensitivity() * int( ( ((int16_t)(Data[1] << 8)) | ( ((int16_t)Data[0]) ) ) >> 2 ); // [g]
  AccelY_ = this->getSensitivity() * int( ( ((int16_t)(Data[3] << 8)) | ( ((int16_t)Data[2]) ) ) >> 2 ); // [g]
  AccelZ_ = this->getSensitivity() * int( ( ((int16_t)(Data[5] << 8)) | ( ((int16_t)Data[4]) ) ) >> 2 ); // [g]
  return true;
}


/**********************************************************************/

/**********************************************************************/
double BMA180::getAccelX()  
{
  uint8_t Data[2];
   
  // Must read LSB first.  MSB and LSB  must be read in one transaction:
  if( this->readSensorData( ADDRESS_ACCLXYZ, Data, 2 ) == false )
  {
    ROS_ERROR("BMA180: cannot read from this protocol.");
    return -9999.9;
  }
  int raw_data = int(( ((int16_t)(Data[1] << 8)) | ( ((int16_t)Data[0]) ) ) >> 2);
  AccelX_ = raw_data * this->getSensitivity();

  return AccelX_;
}



/**********************************************************************/

/**********************************************************************/
double BMA180::getAccelY()
{
  uint8_t Data[2];
 
  if( this->readSensorData( ADDRESS_ACCLY_LSB, Data, 2 ) == false )
  {
    ROS_ERROR("BMA180::getAccelY(): failed.");
    return -9999.9;
  }
  double raw_data = (( ((int16_t)(Data[1] << 8)) | ( (int16_t)(Data[0]) ) ) >> 2);
  AccelY_ = raw_data * this->getSensitivity();
  
  return AccelY_;
}


/**********************************************************************/

/**********************************************************************/
double BMA180::getAccelZ()
{
  uint8_t Data[2];
  if( this->readSensorData( ADDRESS_ACCLZ_LSB, Data, 2 ) == false ) 
  {
    ROS_ERROR( "BMA180: cannot read from this protocol." );
    return -9999.9;
  }
  
  double raw_data = (( ((int16_t)(Data[1] << 8)) | ( (int16_t)(Data[0]) ) ) >> 2);
  AccelZ_ = raw_data * this->getSensitivity();
    
  return AccelZ_;
}


/**********************************************************************/

/**********************************************************************/
double BMA180::getTemperature()
{ 
  uint8_t Temperature;

  if( this->readReg( ADDRESS_TEMPERATURE, &Temperature ) == false )
  {
    ROS_ERROR("BMA180::getTemperature(): failed.");
    return -9999.9;
  }
  // convert raw temperature to actual temperature:
  Temperature_ = ( TempSlope_* (int8_t)Temperature ) + 24;   
  return Temperature_;
}


/**********************************************************************/

/**********************************************************************/
bool BMA180::softReset()
{
// not sure why the 5th argument has to be set in this way:
  uint8_t Request_SoftReset = (uint8_t)CMD_SOFTRESET;
 
  // write 0xB6 to 0x10
  if( this->writeToReg( ADDRESS_SOFTRESET, Request_SoftReset ) == false )
  {
    ROS_ERROR("BMA180::softReset(): write failed.");
    return false;
  }      

  // mandatory wait --- see datasheet  page 49
  usleep( 10 );

  return true;
}


/**********************************************************************/
// perform a fine-calibration ---  see datahseet page 44.
/**********************************************************************/
bool BMA180::Calibrate()
{
  uint8_t bitFlags[3] = { en_offset_x, en_offset_y, en_offset_z };
  uint8_t oldOffsets[3];
  uint8_t newOffsets[3];
 
  for( int i = 0; i < 3; i++ )
  {
    ROS_INFO( "BMA180::Calibrate():  BEGIN  Calibrating axis %d", i );
  
    // extract offsets before: 
    // read fine_offset_x
    if( i == 0 )
    {
      if( this->readReg( 0x29, &oldOffsets[i] ) == false )
        return false;
      oldOffsets[i] = (oldOffsets[i] >> 1) & 0x7F; // see datasheet page 44 for more details.
    }
  
    // read fine_offset_y
    if( i == 1 )
    {
      if( this->readReg( 0x26, &oldOffsets[i] ) == false)
        return false;
      oldOffsets[i] = (oldOffsets[i] >> 1) & 0x7F; // see datasheet page 44 for more details.
    }   
  
    // read fine_offset_z:  it's stored in two registers...
    if( i == 2 )
    {
      uint8_t temp_reg1;
      uint8_t temp_reg2;
   
      if( this->readReg( 0x25, &temp_reg1 ) == false )
        return false;
      if( this->readReg( 0x23, &temp_reg2 ) == false )
        return false; 
      oldOffsets[i] = ( (temp_reg1 << 3) | (0x07 & temp_reg2) ) & 0x7F; // see datasheet page 44 for more details.
    } 
  
    do
    {   
      // Set Fine Calibration settings:
      uint8_t local_reg;
      // read reg for a local copy:
      if( this->readReg( ADDRESS_CTRL_REG4, &local_reg ) == false )
        return false;
   
      // remove old reg settings:
      local_reg &= ~0x03; // Mask: b11111100 
      // write modified local_reg to the sensor and expect local_reg back from the sensor:
      if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): failed." );
        return false;
      } 
      //ROS_INFO("BMA180::Calibrate(...): old calibration settings cleared.");
   
   
      // apply new settings:
      local_reg |= (0x01 << offset_finetuning); // apply fine-calibration flags --- see datasheet page 43.
   
      if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): failed." );
        return false;
      } 
      //ROS_INFO("BMA180::Calibrate(...): fine-calibration accepted");

   
      // enable the offset bit. Calibration begins afterwards.
      if( this->setEnOffsetBit( bitFlags[i] ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): could not set bitFlag to begin calibration." );
        return false;
      }
 
      // delay for two seconds while calibration takes place:
      sleep( 2 );  
  
      // extract offsets after: 
      // read fine_offset_x
      if( i == 0 )
      {
        if( this->readReg( 0x29, &newOffsets[i] ) == false )
          return false;
        newOffsets[i] = (newOffsets[i] >> 1) & 0x7F;
      }
  
      // read fine_offset_y
      if( i == 1 )
      {
        if( this->readReg( 0x29, &newOffsets[i] ) == false )
          return false;
        newOffsets[i] = (newOffsets[i] >> 1) & 0x7F;
      }   
  
      // read fine_offset_z:  it's stored in two registers...
      if( i == 2 )
      {
        uint8_t temp_reg1;
        uint8_t temp_reg2;
   
        if( this->readReg( 0x25, &temp_reg1 ) == false )
          return false;
        if( this->readReg( 0x23, &temp_reg2 ) == false )
          return false;
    
        newOffsets[i] = ((temp_reg1 << 3)|(0x07 & temp_reg2)) & 0x7F;   // See datasheet page 44 for more details. 
   
      }
  
      // Calibration is now complete on that axis.
      ROS_INFO( "Old Offset: %x  New Offset: %x", oldOffsets[i], newOffsets[i] );
    } while( (newOffsets[i] == 0x3f) || (newOffsets[i] == 0x40) );    // 0x3f and 0x40 indicate an error.
 
 
    ROS_INFO( "BMA180::Calibrate(): END Calibrating axis %d.", i );
  }
  ROS_INFO( "BMA180::Calibrate(): Calibration COMPLETE." );
 
  return true;
}


/**********************************************************************/
// perform a Quick coarse-calibration --- see datahseet page 44.
/**********************************************************************/
bool BMA180::CoarseCalibrate()
{
  uint8_t bitFlags[3] = { en_offset_x, en_offset_y, en_offset_z };
  int16_t oldOffsets[3];
  int16_t newOffsets[3];
 
  uint8_t local_offset_z;
  uint8_t local_offset_y;
  uint8_t local_offset_x;
  uint8_t local_offset_lsb1;
  uint8_t local_offset_lsb2;
 
  // get old coarse offsets:  they're stored in two registers...
  if( this->readReg( ADDRESS_OFFSET_Z, &local_offset_z ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_Y, &local_offset_y ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_X, &local_offset_x ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_offset_lsb1 ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB2, &local_offset_lsb2 ) == false )
    return false;

  // convert from offset-binary format to two's complement: 
  oldOffsets[0] = int(( ((int16_t)( (local_offset_x ^= 0x80) << 8)) | ( local_offset_lsb1) ) >> 4);
  oldOffsets[1] = int(( ((int16_t)( (local_offset_y ^= 0x80) << 8)) | ( local_offset_lsb2 << 4) ) >> 4);
  oldOffsets[2] = int(( ((int16_t)( (local_offset_z ^= 0x80) << 8)) | ( local_offset_lsb2) ) >> 4);
 

  ROS_INFO( "Old Coarse Offsets:" );
  ROS_INFO( "X: %f", oldOffsets[0] * this->getSensitivity() );
  ROS_INFO( "Y: %f", oldOffsets[1] * this->getSensitivity() );
  ROS_INFO( "Z: %f", oldOffsets[2] * this->getSensitivity() );


  for( int i = 0; i < 3; i++ )
  {
    ROS_INFO( "BMA180::Calibrate():  BEGIN  Calibrating axis %d", i );
 
    // Set Coarse Calibration settings:
    uint8_t local_reg;
    // read reg for a local copy:
    if( this->readReg( ADDRESS_CTRL_REG4, &local_reg ) == false )
      return false;
   
    // remove old reg settings:
    local_reg &= ~0x03; // Mask: b11111100 
    // write modified local_reg to the sensor and expect local_reg back from the sensor:
    if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg ) == false )
    {
      ROS_ERROR( "BMA180::Calibrate(): failed." );
      return false;
    } 
    //ROS_INFO("BMA180::Calibrate(...): old calibration settings cleared.");
   
    // apply new settings:
    local_reg |= 0x02; // apply coarse-calibration flags --- see datasheet page 43.
   
    if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg ) == false )
    {
      ROS_ERROR( "BMA180::Calibrate(): failed." );
      return false;
    } 
    //ROS_INFO("BMA180::Calibrate(...): fine-calibration accepted");
   
    // Calibrate x and y axis normally:
    if( i != 2 ) 
    {// initialize the sensor's calibration routine on that axis:
      if( this->setEnOffsetBit( bitFlags[i] ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): could not set bitFlag to begin calibration." );
        return false;
      }
      // delay for three seconds while calibration takes place:
      sleep( 2 );  
    }
 
    // Calibrate Z axis in slightly alternate manner.  Assume Z axis is in the 1 [g] position:
    if( i == 2 ) 
    {
      // trigger the Z-axis calibration:
      if( this->setEnOffsetBit( bitFlags[i] ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): could not set bitFlag to begin calibration." );
        return false;
      }
      // delay for three seconds while calibration takes place:
      sleep( 2 );
    
      // If successful, the z-offset registers now will compensate for zero, when we want it to adjust for 1 [g].
      // To fix this, calibrate in a 1[g] environment and add the 1[g] equivalent to the calibrated value.
      // See datasheet pg 43.
    
      uint8_t new_offset_z;
      uint8_t new_offset_lsb2;
      // read offset values after calibration:
      if( this->readReg( ADDRESS_OFFSET_Z, &new_offset_z ) == false )
        return false;
      if( this->readReg( ADDRESS_OFFSET_LSB2, &new_offset_lsb2 ) == false )
        return false;
    
      // combine to raw Two's Complement shifted over by 4:
      uint16_t rawTC = ((uint16_t)new_offset_z << 8) |(uint16_t)new_offset_lsb2;
      rawTC ^= 0x8000;
      // add 1[g] equivalent (shifted over, since the other value is still shifted):
      int16_t one_g;
    
      switch( accel_range_ )
      {
      case RANGE_1: {}
      case RANGE_1_5: {}
      case RANGE_2: {}
      case RANGE_4:   
        one_g = (0x009A << 4); // a magic number nearby 8F, which is about 1[g] in +/- 2 [g] mode.  This value comes from 143 * 7[mg].  
        break;
      case RANGE_8: {}
      case RANGE_16:
        one_g = (0x008F << 4);
        break;
      default:  // should never happen
        one_g = (0x008F << 4);
      } 
      // add:
      int16_t sum_ob = (int16_t)rawTC + one_g;
      // convert back to offset-binary format:      
      sum_ob ^= 0x8000;
      // insert back into registers:
      new_offset_z = (uint8_t)(sum_ob >> 8);  
      new_offset_lsb2 = (uint8_t)(0x00FF & sum_ob);
    
      ROS_INFO( "offset_z: %x", new_offset_z );
      ROS_INFO( "offset_lsb2: %x", new_offset_lsb2 );
   
    
      if( this->writeToRegAndVerify( ADDRESS_OFFSET_Z, new_offset_z, new_offset_z ) == false )
        return false;
      if( this->writeToRegAndVerify( ADDRESS_OFFSET_LSB2, new_offset_lsb2, new_offset_lsb2 ) == false )
        return false;
    }
    ROS_INFO( "BMA180::Calibrate(): END     Calibrating axis %d \r\n", i );
  } 
 
  // extract offsets after: they're stored in two registers...
  if( this->readReg( ADDRESS_OFFSET_Z, &local_offset_z ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_Y, &local_offset_y ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_X, &local_offset_x ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_offset_lsb1 ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB2, &local_offset_lsb2 ) == false )
    return false;

  // convert from offset-binary format to two's complement: 
  newOffsets[0] = int(( ((int16_t)( (local_offset_x ^= 0x80) << 8)) | ( local_offset_lsb1) ) >> 4);
  newOffsets[1] = int(( ((int16_t)( (local_offset_y ^= 0x80) << 8)) | ( local_offset_lsb2 << 4) ) >> 4);
  newOffsets[2] = int(( ((int16_t)( (local_offset_z ^= 0x80) << 8)) | ( local_offset_lsb2) ) >> 4);
 
  ROS_INFO( "New Coarse Offsets:" );
  ROS_INFO( "X: %f", newOffsets[0] * this->getSensitivity() );
  ROS_INFO( "Y: %f", newOffsets[1] * this->getSensitivity() );
  ROS_INFO( "Z: %f", newOffsets[2] * this->getSensitivity() );

  ROS_INFO( "BMA180::Calibrate(): COARSE CALIBRATION COMPLETE." );
 
  return true;
}


/**********************************************************************/
// perform a two-step coarse-calibration --- see datahseet page 44.
/**********************************************************************/
bool BMA180::TwoStepCoarseCalibrate()
{
  uint8_t bitFlags[3] = { en_offset_x, en_offset_y, en_offset_z };
  std::string axes[3] = { "X", "Y", "Z" };
  int16_t oldOffsets[3];
  int16_t newOffsets[3];
 
  uint8_t local_offset_z;
  uint8_t local_offset_y;
  uint8_t local_offset_x;
  uint8_t local_offset_lsb1;
  uint8_t local_offset_lsb2;
 
  // get old coarse offsets:  Each axis is stored in two registers.
  if( this->readReg( ADDRESS_OFFSET_Z, &local_offset_z ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_Y, &local_offset_y ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_X, &local_offset_x ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_offset_lsb1 ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB2, &local_offset_lsb2 ) == false )
    return false;

  oldOffsets[0] = ((((local_offset_x << 8) | local_offset_lsb1) >> 4) & 0x0FFF);
  oldOffsets[1] = (((local_offset_y << 8) | (local_offset_lsb2 << 4) >> 4) & 0x0FFF);
  oldOffsets[2] = ((((local_offset_z << 8) | local_offset_lsb2) >> 4) & 0x0FFF);
 
  for( int i = 0; i < 3; i++ )
  {
    ROS_INFO( "BEGIN Calibrating axis %s.", axes[i].c_str() );
    // Set Coarse Calibration settings:
    uint8_t local_reg;
    // read reg for a local copy:
    if( this->readReg( ADDRESS_CTRL_REG4, &local_reg ) == false )
      return false;
   
    // remove old reg settings:
    local_reg &= ~0x03; // Mask: b11111100 
    // write modified local_reg to the sensor and expect local_reg back from the sensor:
    if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg )  == false )
    {
      ROS_ERROR( "BMA180::TwoStepCoarseCalibrate(): failed." );
      return false;
    } 
   
    // apply new settings:
    local_reg |= 0x02; // apply coarse-calibration flags --- datasheet page 43
   
    if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg )  == false )
    {
      ROS_ERROR( "BMA180::TwoStepCoarseCalibrate(): failed." );
      return false;
    } 

    // Calibrate x and y axis normally:
    if( i != 2 ) 
    {
      if( i == 0 )
      {
        // Query user:
        // Place the sensor on a still flat surface.
        std::cout << std::endl << "Place the sensor on a still, flat surface." << std::endl;
        std::cout << "Ready to calibrate x and y axes. Continue? [y/n]" << std::endl;
        std::string reply;
        std::getline( std::cin, reply );
        if( reply != "Y" && reply != "y" )
        {
          ROS_INFO( "Calibration cancelled by user." );
          return true;
        }
      }
    
      // initialize the sensor's calibration routine on that axis:
      if( this->setEnOffsetBit( bitFlags[i] ) == false )
      {
        ROS_ERROR( "BMA180::TwoStepCoarseCalibrate(): could not set bitFlag to begin calibration." );
        return false;
      }
      // delay for two seconds while calibration takes place:
      sleep( 2 );
    }
 
    // Calibrate Z axis in slightly alternate manner:
    if( i == 2 ) 
    {
      // Query user again:
      std::cout << std::endl << "Ready to calibrate z axis." << std::endl
		<< "Orient sensor such that z-axis is perpendicular to gravity." << std::endl
		<< "Do not move the sensor in this position." <<std::endl
		<< "Enter [y/n] to continue/cancel calibration." << std::endl;
      std::string reply;
      std::getline( std::cin, reply );
      if( reply != "Y" && reply != "y" )
      {
        ROS_INFO( "Calibration cancelled by user." );
        return false;
      }
 
      // trigger the Z-axis calibration:
      if( this->setEnOffsetBit( bitFlags[i] ) == false )
      {
        ROS_ERROR( "BMA180::TwoStepCoarseCalibrate(): could not set bitFlag to begin calibration." );
        return false;
      }
      // delay for two seconds while calibration takes place:
      sleep( 2 );
    }
   
    ROS_INFO( "BMA180::TwoStepCoarseCalibrate(): END     Calibrating axis %d.", i );
  } 

  // get new coarse offsets:  they're stored in two registers...
  if( this->readReg( ADDRESS_OFFSET_Z, &local_offset_z ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_Y, &local_offset_y ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_X, &local_offset_x ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_offset_lsb1 ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB2, &local_offset_lsb2 ) == false )
    return false;

  newOffsets[0] = ((((local_offset_x << 8)|local_offset_lsb1) >> 4) & 0x0FFF);
  newOffsets[1] = (((local_offset_y << 8)|(local_offset_lsb2 << 4) >> 4) & 0x0FFF);
  newOffsets[2] = ((((local_offset_z << 8)|local_offset_lsb2) >> 4) & 0x0FFF);
 

  std::cout << "(in 12-bit offset-binary format)" << std::endl;
  std::cout << "Old Coarse Offsets   |   New Coarse Offsets " << std::endl;
  std::cout << "            " << std::hex << oldOffsets[0]
	    << "            " << std::hex << newOffsets[0] << std::endl;
  std::cout << "            " << std::hex << oldOffsets[1]
	    << "            " << std::hex << newOffsets[1] << std::endl;
  std::cout << "            " << std::hex << oldOffsets[2]
	    << "            " << std::hex << newOffsets[2] << std::endl;

  ROS_INFO( "BMA180::Calibrate(): COARSE CALIBRATION COMPLETE." );
 
  return true;
}


/**********************************************************************/
// perform a full calibration --- see datahseet page 44.
/**********************************************************************/
bool BMA180::FullCalibration()
{
  uint8_t bitFlags[3] = { en_offset_x, en_offset_y, en_offset_z };
  std::string axes[3] = { "X", "Y", "Z" };
  // coarse calibration:
  int16_t oldOffsets[3];
  int16_t newOffsets[3];
  // fine calibration:
  uint8_t oldFineOffsets[3];
  uint8_t newFineOffsets[3];
 
  uint8_t local_offset_z;
  uint8_t local_offset_y;
  uint8_t local_offset_x;
  uint8_t local_offset_lsb1;
  uint8_t local_offset_lsb2;
 
  // get old coarse offsets:  Each axis is stored in two registers.
  if( this->readReg( ADDRESS_OFFSET_Z, &local_offset_z ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_Y, &local_offset_y ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_X, &local_offset_x ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_offset_lsb1 ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB2, &local_offset_lsb2 ) == false )
    return false;

  oldOffsets[0] = ((((local_offset_x << 8)|local_offset_lsb1) >> 4) & 0x0FFF);
  oldOffsets[1] = (((local_offset_y << 8)|(local_offset_lsb2 << 4) >> 4) & 0x0FFF);
  oldOffsets[2] = ((((local_offset_z << 8)|local_offset_lsb2) >> 4) & 0x0FFF);
 
  // get old fine offsets: Each offset is stored in two registers:   // see datasheet page 44 for more details.... 
  if( this->readReg( 0x29, &oldFineOffsets[0] ) == false )
    return false;
  oldFineOffsets[0] = (oldFineOffsets[0] >> 1) & 0x7F;         
 
  if( this->readReg( 0x26, &oldFineOffsets[1] ) == false )
    return false;
  oldFineOffsets[1] = (oldFineOffsets[1] >> 1) & 0x7F;  
 
  uint8_t temp_reg1, temp_reg2;
  if( this->readReg( 0x25, &temp_reg1 ) == false )
    return false;
  if( this->readReg( 0x23, &temp_reg2 ) == false )
    return false;  
  oldFineOffsets[2] = ((temp_reg1 << 3)|(0x07 & temp_reg2)) & 0x7F;   

  for( int i = 0; i < 3; i++ )
  {
    ROS_INFO( "BEGIN Calibrating axis %s.", axes[i].c_str() );
  
    do
    {   
      // Set Calibration settings:
      uint8_t local_reg;
      // read reg for a local copy:
      if( this->readReg( ADDRESS_CTRL_REG4, &local_reg ) == false )
        return false;
    
      // remove old reg settings:
      local_reg &= ~0x03; // Mask: b11111100 
      // write modified local_reg to the sensor and expect local_reg back from the sensor:
      if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): failed." );
        return false;
      }    
    
      // apply new settings:
      local_reg |= 0x03; // apply fine-calibration flags --- datasheet page 43.
    
      if( this->writeToRegAndVerify( ADDRESS_CTRL_REG4, local_reg, local_reg ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): failed." );
        return false;
      } 

      // Calibrate x and y axis normally:
      if( i == 0 )
      {
        // Query user:
        // Place the sensor on a still flat surface.
        std::cout << std::endl << "Place the sensor on a still, flat surface." << std::endl
		  << "Ready to calibrate x and y axes. Continue? [y/n]" << std::endl;
        std::string reply;
        std::getline( std::cin, reply );
        if( reply != "Y" && reply != "y" )
        {
          ROS_INFO( "Full Calibration cancelled by user." );
          return false;
        }
      }
    
      // Calibrate Z axis in slightly alternate manner:
      if( i == 2 )
      {
        // Query user again:
        std::cout << std::endl << "Ready to calibrate z-axis." << std::endl
		  << "Orient sensor such that z-axis is perpendicular to gravity." << std::endl
		  << "Do not move the sensor in this position." << std::endl 
		  << "Enter [y/n] to continue/cancel calibration." << std::endl;
        std::string reply;
        std::getline( std::cin, reply );
        if ( reply != "Y" && reply != "y" )
        {
          ROS_INFO( "Full Calibration cancelled by user." );
          return false;
        }
      }

      // enable the offset bit. Calibration begins afterwards.
      if( this->setEnOffsetBit( bitFlags[i] ) == false )
      {
        ROS_ERROR( "BMA180::Calibrate(): could not set bitFlag to begin calibration." );
        return false;
      }
      // delay for two seconds while calibration takes place:
      sleep( 2 );
   
      // extract newFineoffsets after: 
      // read fine_offset_x
      if( i == 0 )
      {
        if( this->readReg( 0x29, &newFineOffsets[i] ) == false )
          return false;
        newFineOffsets[i] = (newFineOffsets[i] >> 1) & 0x7F;
      }
      // read fine_offset_y
      if( i == 1 )
      {
        if( this->readReg( 0x29, &newFineOffsets[i] ) == false )
          return false;
        newFineOffsets[i] = (newFineOffsets[i] >> 1) & 0x7F;
      }   
      // read fine_offset_z:  it's stored in two registers...
      if( i == 2 )
      {
        uint8_t temp_reg1, temp_reg2;
    
        if( this->readReg( 0x25, &temp_reg1 ) == false )
          return false;
        if( this->readReg( 0x23, &temp_reg2 ) == false )
          return false;
        newFineOffsets[i] = ((temp_reg1 << 3)|(0x07 & temp_reg2)) & 0x7F;   // see datasheet page 44 for more details.
      }
      // if fineoffset values remain within range,
      // calibration is now complete on that axis. 
      // Do..while().. will restart calibration on that axis otherwise.
      ROS_INFO( "NewFineOffsets: %x", newFineOffsets[i] );
   
    } while( (newFineOffsets[i] == 0x3f) || (newFineOffsets[i] == 0x40) );    // 0x3f and 0x40 indicate an error.
    
    ROS_INFO( "BMA180::FullCalibration(): END     Calibrating axis %d.", i );
  } 

  // get new coarse offsets:  they're stored in two registers...
  if( this->readReg( ADDRESS_OFFSET_Z, &local_offset_z ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_Y, &local_offset_y ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_X, &local_offset_x ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_offset_lsb1 ) == false )
    return false;
  if( this->readReg( ADDRESS_OFFSET_LSB2, &local_offset_lsb2 ) == false )
    return false;

  newOffsets[0] = ((((local_offset_x << 8)|local_offset_lsb1) >> 4) & 0x0FFF);
  newOffsets[1] = (((local_offset_y << 8)|(local_offset_lsb2 << 4) >> 4) & 0x0FFF);
  newOffsets[2] = ((((local_offset_z << 8)|local_offset_lsb2) >> 4) & 0x0FFF);
 
  std::cout << "(in 12-bit offset-binary format)" << std::endl;
  std::cout << "Old Coarse Offsets   |   New Coarse Offsets " << std::endl;
  std::cout << "X:          " << std::hex << oldOffsets[0]
	    << "            " << std::hex << newOffsets[0] << std::endl;
  std::cout << "Y:          " << std::hex << oldOffsets[1]
	    << "            " << std::hex << newOffsets[1] << std::endl;
  std::cout << "Z:          " << std::hex << oldOffsets[2]
	    << "            " << std::hex << newOffsets[2] << std::endl;

  std::cout << "(in 7-bit offset-binary format)" << std::endl;
  std::cout << "Old Fine Offsets   |   New Fine Offsets " << std::endl;
  std::cout << "X:        " << std::hex << (int)oldFineOffsets[0]
	    << "          " << std::hex << (int)newFineOffsets[0] << std::endl;
  std::cout << "Y:        " << std::hex << (int)oldFineOffsets[1]
	    << "          " << std::hex << (int)newFineOffsets[1] << std::endl;
  std::cout << "Z:        " << std::hex << (int)oldFineOffsets[2]
	    << "          " << std::hex << (int)newFineOffsets[2] << std::endl;

  ROS_INFO( "BMA180::FullCalibration(): FULL CALIBRATION COMPLETE." );
 
  return true;
}


/**********************************************************************/

/**********************************************************************/
bool BMA180::DisableI2C()
{
  // set the LSB of ADDRESS_HIGH_DUR high.
  uint8_t ctrl_register = (uint8_t)CMD_DISABLE_I2C;

  switch( this->getProtocol() )
  {
  case SPI:
    if( this->writeToReg( ADDRESS_HIGH_DUR, ctrl_register ) == false )
    {
      ROS_ERROR( "BMA180::DisableI2C(): write failed." );
      return false;
    }
    return true;
  default:
    ROS_ERROR( "BMA180::DisableI2C() cannot disable i2c from this protocol." );
  }
  return false;
}


/**********************************************************************/

/**********************************************************************/
bool BMA180::EnableWriting()
{
  uint8_t reg_value;
 
  // Read the register that we want to write to first for a local copy
  if( this->readReg( ADDRESS_CTRLREG0, &reg_value ) == false )
  {
    ROS_ERROR( "BMA180::EnableWriting(): read failed." );
    return false;
  }
  
  // #ifdef DEBUG
  ROS_INFO( "EnableWriting flag before: %d. Default: 0", (reg_value & (1 << ee_w) ) >> ee_w );
  // #endif  
 
  // add our command to enable writing:
  reg_value |= (1 << ee_w); // NOTE: for other cases where we write to bits, we must mask off the existing bits.
 
  // write the adjusted register value back to the sensor:
  if( this->writeToReg( ADDRESS_CTRLREG0, reg_value ) == false )
  {
    ROS_ERROR( "BMA180::EnableWriting(): write failed." );
    return false;
  }
 
  // read back that register to make sure that changes worked:
  if( this->readReg( ADDRESS_CTRLREG0, &reg_value ) == false )
  {
    ROS_ERROR( "BMA180::EnableWriting(): read failed." );
    return false;
  }
 
  // Compare register values to what we expect:
  uint8_t enable_actual = ( reg_value & (1 << ee_w) ) >> ee_w;
  uint8_t enable_expected = 1; 
 
  // #ifdef DEBUG
  ROS_INFO("EnableWriting flag after:  %d.  Expected: %d", enable_actual, enable_expected );
  // #endif 
 
  if( enable_expected != enable_actual )
  {
    ROS_ERROR( "BMA180::EnableWriting(): failed." );
    return false;
  }
 
  // if new settings are correct:
  return true;
}


/**********************************************************************/

/**********************************************************************/
bool BMA180::setAccelerationRange(accel_range measurement_range )
{
  uint8_t local_range;
 
  // read current accel range register value for a local copy.
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_range ) == false )
  {
    ROS_ERROR("bma180_driver: setAccelerationRange() failed to read current range.");
    return false;
  }
 
  ROS_DEBUG( "bma180_driver: Acceleration range bits before changing: %d.  Default:  %d", ( (local_range & (0x07 << range)) >> range), 2); // Defaults on page 27 of datasheet

  // add our command to change range:
  local_range &= ~(0x07 << range); // clear old range value. Mask: b11110001
  local_range |= (accel_range_ << range); // insert new range value.

  // write the adjusted register value back to the sensor's register:
  if( this->writeToReg( ADDRESS_OFFSET_LSB1, local_range ) == false )
  {
    ROS_ERROR( "bma180_driver: setAccelerationRange() failed to write new range to sensor." );
    return false;
  }
 
  // read back that register to make sure that changes worked:
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_range ) == false )
  {
    ROS_ERROR( "bma180_driver: setAccelerationRange() failed to read new range from sensor." );
    return false;
  }
 
  // Compare register values to what we expect:
  uint8_t range_actual = ( local_range & (0x07 << range) ) >> range; // mask: b00001110
  uint8_t range_expected = (uint8_t) accel_range_; // This is the value set in the parameters. 
 
  ROS_DEBUG( "bma180_driver: Acceleration range bits after change:  %d.  Expected: %d", range_actual, range_expected );
 
  if( range_expected != range_actual )
  {
    ROS_ERROR( "bma180_driver: setAccelerationRange() failed verification step." );
    return false;
  }


  // After changing the sensor, change sensitivity
  switch( measurement_range )
  {
  case RANGE_1:
    sensitivity_ = 0.00013;
    break;
  case RANGE_1_5:
    sensitivity_ = 0.00019;
    break;
  case RANGE_2:
    sensitivity_ = 0.00025;
    break;
  case RANGE_3:
    sensitivity_ = 0.00038;
    break;
  case RANGE_4:
    sensitivity_ = 0.00050;
    break;
  case RANGE_8:
    sensitivity_ = 0.00099;
    break;
  case RANGE_16:
    sensitivity_ = 0.00198;
    break;
  default: // shouldn't happen because input argument is only an accel_range data type.
    ROS_ERROR( "bma180_parameters: invalid range setting." );
    return false;
  }

 
  // if new settings are correct:
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA180::changeBandwidth()
{
  uint8_t local_bw_reg;
  // read register with the current bandwidth flags for a local copy.
  if( this->readReg( ADDRESS_BW_TCS, &local_bw_reg ) == false )
  {
    ROS_ERROR( "BMA180::changeBandwidth(): read failed." );
    return false;
  }
 
  // #ifdef DEBUG
  ROS_INFO( "Bandwidth bits before: %d.  Default:  %d", ( (local_bw_reg & (0x0F << bw)) >> bw), 4 ); // defaults on page 27
  // #endif 
 
  // add our command to change range:
  local_bw_reg &= ~(0x0F << bw); // clear old value. Mask: b00001111
  local_bw_reg |= bandwidth_ << bw; // insert new value.

  // write the adjusted register value back to the sensor's register
  if( this->writeToReg( ADDRESS_BW_TCS, local_bw_reg ) == false )
  {
    ROS_ERROR( "BMA180::changeBandwidth(): write failed." );
    return false;
  }
 
  // read back that register to make sure that changes worked:
  if( this->readReg( ADDRESS_BW_TCS, &local_bw_reg ) == false )
  {
    ROS_ERROR( "BMA180::changeBandwidth(): read failed." );
    return false;
  }
 
  // Compare register values to what we expect:
  uint8_t bandwidth_actual = ( local_bw_reg & (0x0F << bw) ) >> bw; // mask: b11110000
  uint8_t bandwidth_expected = (uint8_t) bandwidth_; // This is the value set in the parameters. 
 
  // #ifdef DEBUG
  ROS_INFO("Bandwidth bits after:  %d.  Expected: %d", bandwidth_actual, bandwidth_expected);
  // #endif 
 
  if( bandwidth_expected != bandwidth_actual )
  {
    ROS_ERROR( "BMA180::changeBandwidth(): failed." );
    return false;
  }
 
  // if new settings are correct:
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA180::setEnOffsetBit( uint8_t bit )
{
  uint8_t local_ctrl_reg1;
 
  // BEGIN ENABLE OFFSET on selected axis: 
  // read CTRL_REG1 register value for a local copy.
  if( this->readReg( ADDRESS_CTRLREG1, &local_ctrl_reg1 ) == false )
  {
    ROS_ERROR( "BMA180::setEnOffsetBit(): read failed." );
    ROS_INFO("en_offset bit before:  %d.  Default:  0", ((local_ctrl_reg1 >> bit) & 0x01) );
    return false;
  }
  // #ifdef DEBUG 
  //ROS_INFO("en_offset bit before:  %d.  Default:  0", ((local_ctrl_reg1 >> bit) & 0x01) );
  // #endif 
  // add our command to change range:
  local_ctrl_reg1 &= ~(1 << bit);   // clear old  value. Mask: b01111111
  local_ctrl_reg1 |= (1 << bit); // insert new  value.
 
  // write the adjusted register value back to the sensor's register:
  if( this->writeToReg( ADDRESS_CTRLREG1, local_ctrl_reg1 ) == false )
  {
    ROS_ERROR( "BMA180::setEnOffsetBit(): write failed." );
    return false;
  }
  
  // read back that register to make sure that changes worked:
  if( this->readReg( ADDRESS_CTRLREG1, &local_ctrl_reg1 ) == false )
  {
    ROS_ERROR( "BMA180::setEnOffsetBit(): read failed." );
    return false;
  }
 
  // Compare register values to what we expect:
  uint8_t actual = ( local_ctrl_reg1 >> bit ) & 0x01;     
  uint8_t expected = 1; // flag should be set to 1.
   
  // #ifdef DEBUG
  //ROS_INFO("en_offset bit after:   %d.  Expected: %d", actual, expected);
  // #endif
  
  if( expected != actual )
  {
    ROS_ERROR( "BMA180::setEnOffsetBit(): failed." );
    ROS_INFO( "en_offset bit after:   %d.  Expected: %d", actual, expected );
    return false;
  }
  // if new settings are correct:
  return true;
}

/**********************************************************************/
// read a register and return its value. 
/**********************************************************************/
bool BMA180::readReg( uint8_t reg, uint8_t* value )
{
  std::vector<uint8_t> data(1);

  // Reading depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->read( *sensor_parameters_, reg, data ) < 0 ) 
    {
      ROS_ERROR( "bma180_driver: Error reading register via I2C!" );
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG.
    if( hardware_->read( *sensor_parameters_, ( 1 << SPI_READ_FLAG ) | reg, data ) < 0 ) 
    {
      ROS_ERROR( "bma180_driver: Error reading register via SPI!" );
      return false;
    } 
    break;
  default:
    // shouldn't happen:
    ROS_ERROR( "bma180_driver:readReg(...): invalid protocol." );
    return false;
  }

  *value = data[0];
  return true;
}

/**********************************************************************/
// writes a byte to a register.
/**********************************************************************/
bool BMA180::writeToReg( uint8_t reg, uint8_t value )
{
  std::vector<uint8_t> data(1,value);

  // technically, writing depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->write( *sensor_parameters_, reg, data ) < 0 )
    {
      ROS_ERROR( "bma180_driver: Error writing to register via I2C!" );
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_WRITE_FLAG, although, technically it's already there, since it's zero.
    if( hardware_->write( *sensor_parameters_, (~(1 << SPI_WRITE_FLAG)&reg), data) < 0 ) 
    {
      ROS_ERROR( "bma180_driver: Error writing to register via SPI!" );
      return false;
    } 
    break;
  default:
    ROS_ERROR("bma180_driver: invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
// writes a byte to a register.
/**********************************************************************/
bool BMA180::writeToRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected )
{
  uint8_t actual;
 
  if( this->writeToReg( reg, value ) == false )
    return false;
  // read it back to make sure it worked.
  if( this->readReg( reg, &actual ) == false )
    return false;

  if( expected != actual )
  {
    ROS_ERROR( "BMA180::writeToRegAndVerify(...): failed." );
    ROS_ERROR( "(in Hex) expected:  %x  actual: %x", expected, actual );
    return false;
  }

  return true;
}


bool BMA180::readSensorData( uint8_t reg, uint8_t* sensor_data, uint8_t num_bytes )
{
  std::vector<uint8_t> data(num_bytes);
  // Reading depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->read( *sensor_parameters_, reg, data ) < 0 ) 
    {
      ROS_ERROR( "bma180_driver: Error reading register via I2C!" );
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG.
    if( hardware_->read( *sensor_parameters_, ((1 << SPI_READ_FLAG)|reg), data ) < 0 ) 
    {
      ROS_ERROR( "bma180_driver: Error reading register via SPI!" );
      return false;
    } 
    break;
  default:
    // shouldn't happen:
    ROS_ERROR( "bma180_driver: invalid protocol." );
    return false;
  }

  sensor_data = &data[0];
  return true; 
}


/**********************************************************************/
/**********************************************************************/
bool BMA180::setProtocol( interface_protocol protocol )
{
  switch( protocol )
  {
  case I2C:
  case SPI:
    sensor_parameters_->protocol = protocol;
    break;
  default:
    ROS_ERROR( "bma180_parameters:Unsupported protocol." );
    return false;
  }
  return true;
}

/**********************************************************************/
/**********************************************************************/
bool BMA180::setByteOrder( uint8_t value )
{
  // adjust the flags
  sensor_parameters_->flags = ( (0xFB & sensor_parameters_->flags) | (value << 2) );
  if( value > 1 )
  {
    ROS_ERROR("bma180_driver: Byte order must be either LSB_FIRST or MSB_FIRST");
    return false;
  }
  return true;
 
}

/**********************************************************************/
/**********************************************************************/
bool BMA180::setSpiMode( uint8_t mode )
{
  // adjust the flags
  sensor_parameters_->flags = ( (0xFC & sensor_parameters_->flags) | (mode) ); // 111111xx, where xx is the mode.
 
  switch( mode )
  {
  case SPI_MODE_3:
    return true;
  case SPI_MODE_0:
  case SPI_MODE_1:
  case SPI_MODE_2:
  default:
    ROS_ERROR( "bma180_driver: BMA180 can only be read in SPI_MODE_3." );
    return false;
  }
}


/**********************************************************************/
/**********************************************************************/
void BMA180::setBandwidth( bandwidth bw )
{ 
  bandwidth_ = bw;
}


/**********************************************************************/
/**********************************************************************/
double BMA180::getSensitivity()
{
  return sensitivity_;
}


/**********************************************************************/
/**********************************************************************/
void BMA180::setPreCalOffsets( bool choice )
{
  offsetsEnabled_ = choice;
}
