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

#include "bmc050_driver/bmc050.hpp"
#include "bmc050_driver/bmc050_parameters.hpp"


// Constructor
BMC050::BMC050( bosch_hardware_interface* hw ) :
  sensor_driver( hw ),
  BMC050Parameters()
{
  // Compass sensitivity is nominally 0.3 uT/LSB at 20 deg C
  // The compass measurements are not calibrated or temperature compensated
  SensitivityXY_ = 0.3; // [uT/LSB]
  SensitivityZ_ = 0.3;  // [uT/LSB]
  TempSlope_ = 0.5;
 
  prev_bw_reg_ = BW_1000HZ; // sensor default value.
}

// Destructor
BMC050::~BMC050() 
{
}


/**********************************************************************/
// Initialize sensor and hardware on requested parameters:
/**********************************************************************/
bool BMC050::initialize()
{  
  // Initialize the hardware interface with the selected parameters.
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("bmc050::initialize(): Could not initialize a hardware interface!");
    return false;
  }

  this->softResetCompass(); // Compass remains in suspend mode if softReset is applied and it is already in suspend mode. pg 73
  this->softResetAccel();
  // Compass sensor starts in suspend_ mode;
  // Accelerometer sensor starts in normal_ mode.
  
  usleep( 3000 ); // manditory wait for sensor startup

  // copy existing values of Compass registers into the class values:
  // (so we don't override existing values when we change modes by setting flags)
  if( this->readRegistersAccel() == false )
    return false;
  if( this->readRegistersCompass() == false )    
    return false;
 
  // Enter Normal mode for accessing sensor data. 
  if( this->enterNormalModeAccel() == false )
    return false;
  if( this->enterNormalModeCompass() == false )
    return false;
  this->readRegistersAccel();
  // Normal mode:  Accel reg 0x11 should have val 0x00
  this->readRegistersCompass();
  // Normal mode:  Compass reg 0x4C should have val 0x00
 

  // change accel_range to the range set in the parameters class:   
  ROS_INFO("bmc050::initialize(): adjusting Accelerometer range.");
  if( this->changeAccelRange() == false )
    return false;
  ROS_INFO("bmc050::initialize(): Accelerometer range adjusted.");
  // change compass_rate to the rate set in the parameters class: 
  ROS_INFO("bmc050::initialize(): adjusting Compass output data rate.");
  if( this->changeCompassRate() == false )
    return false;
  ROS_INFO("bmc050::initialize(): Compass rate adjusted.");
  // filter data at requested bandwidth, if requested in parameters:
  ROS_INFO("bmc050::initialize(): Applying Filter parameter.");
  if( this->filterData(accel_is_filtered_) == false )
    return false;
  ROS_INFO("bmc050::initialize(): Filter parameter applied.");
  // change accelerometer bandwidth to the bandwidth set in the parameters:
  ROS_INFO("bmc050::initialize(): Adjusting Accelerometer Bandwidth.");
  if( this->changeAccelBandwidth() == false )
    return false;
  ROS_INFO("bmc050::initialize(): Bandwidth adjusted.");
  // change compass repetitions register to reflect # reps set in parameter:
  ROS_INFO("bmc050::initialize(): Adjusting Compass X,Y,Z number of samples.");
  if( this->changeNumRepetitionsXY() == false )
    return false;
  if( this->changeNumRepetitionsZ() == false )
    return false;
  if( this->readRegistersAccel() == false )
    return false;
  if( this->readRegistersCompass() == false )
    return false;
 
  return true;
}



/**********************************************************************/

/**********************************************************************/
bool BMC050::softResetAccel()
{
  // Write 0xB6 to Accel Reg 0x14:
  if( this->writeToAccelReg( SOFTRESET_ACCEL, SOFTRESET_ACCEL_CMD ) == false )
  {
    ROS_ERROR("bmc050::softResetAccel(): failed.");
    return false;
  }
 
  usleep( 2000 ); // mandatory delay --- see datasheet page 8.
  accel_mode_ = accel_normal_; 
 
  // clear accel bandwidth settings to sensor default:
  prev_bw_reg_ = BW_1000HZ;
 
  return true;
}


bool BMC050::softResetCompass()
{
  if( this->writeToAccelReg( SOFTRESET_COMPASS_REG, SOFTRESET_COMPASS_CMD ) == false )
  {
    ROS_ERROR("bmc050::softResetCompass(): failed.");
    return false;
  } 

  usleep( 1000 ); // mandatory delay --- see datasheet page 10.

  // compass starts in suspend mode;
  compass_mode_ = compass_suspend_;
 
  return true;
}

/**********************************************************************/
/* INPUT: none
 *OUTPUT: boolean indicating success.
 *  This method changes the sensor's output data rate to the data rate
 *  specified in the parameters by writing the specified value to the 
 *  correct register.
 */
/**********************************************************************/
bool BMC050::changeCompassRate()
{
  uint8_t local_rate;

  // Read register with accel range settings for a local copy:
  if( this->readRegCompass(RATE_COMPASS, &local_rate ) == false )
    return false;
  
  // #ifdef DEBUG                            
  ROS_INFO("Compass rate bits before: %d.  Default: %d", ((local_rate >> Data_Rate) & 0x07), 0); // Defaults from page 50
  // #endif 
  
  // add our command to change range:
  local_rate &= 0xC7; // clear old range value. Mask: b11000111
 
  // set compass rate value to write:
  switch( compass_rate_ )
  {
  case ODR_10HZ:
    local_rate |= (ODR_10HZ << Data_Rate); // (ODR_10HZ << 3) | 11000111b
    break;
  case ODR_2HZ: 
    local_rate |= (ODR_2HZ << Data_Rate);
    break;
  case ODR_6HZ: 
    local_rate |= (ODR_6HZ << Data_Rate);
    break;
  case ODR_8HZ: 
    local_rate |= (ODR_8HZ << Data_Rate);
    break;
  case ODR_15HZ:
    local_rate |= (ODR_15HZ << Data_Rate);
    break;
  case ODR_20HZ:
    local_rate |= (ODR_20HZ << Data_Rate);
    break;
  case ODR_25HZ:
    local_rate |= (ODR_25HZ << Data_Rate);
    break;
  case ODR_30HZ:
    local_rate |= (ODR_30HZ << Data_Rate);
    break;
  default:
    // should never happen because of the enums
    ROS_ERROR("bmc050::changeCompassRate(): invalid compass rate.");
    return false;
  }
 
 
  // write the adjusted register value back to the sensor's register:
  if( this->writeToCompassReg(RATE_COMPASS, local_rate ) == false )
    return false;
  
  // read back that register to make sure that changes worked:
  if( this->readRegCompass(RATE_COMPASS, &local_rate ) == false )
    return false;
  
  // Compare register values to what we expect:
  uint8_t expected_reg = compass_rate_ ;
  uint8_t actual_reg = (local_rate >> Data_Rate) & 0x07;
  
  if( expected_reg != actual_reg )
  {
    ROS_ERROR("Compass rate bits after:  %d.  Expected: %d", actual_reg, expected_reg);
    return false;
  }
  else
  {
    // #ifdef DEBUG                            
    ROS_INFO("Compass rate bits after:  %d.  Expected: %d", actual_reg, expected_reg);
    // #endif
  }
  // if everything works:
  return true;
}


/**********************************************************************/
/* INPUT: none
 *OUTPUT: boolean indicating success.
 *  Because we MUST write an entire register to the accelerometer when
 *  setting bitflags, this function saves a copy of those registers 
 *  so that we write our modified copy of that register rather than
 *  overwriting the sensor's entire register just to set a single bit.
 */
/**********************************************************************/
bool BMC050::readRegistersAccel()
{
  // reading depends on the protocol:
  uint8_t mode_change_accel_reg;
  uint8_t set_filter_reg;

  switch( protocol_ )
  {
  case I2C:
    // normal registers
    mode_change_accel_reg  = MODECHANGE_ACCEL;
    set_filter_reg = SET_FILTER;
    break;
  case SPI:
    // registers with the FLAG_R inserted
    mode_change_accel_reg  = FLAG_R | MODECHANGE_ACCEL;
    set_filter_reg = FLAG_R | SET_FILTER;
    break;
  default:
    ROS_ERROR("bmc050::readRegistersAccel(): invalid protocol.");
    return false;
  }
  // copy MODECHANGE_ACCEL located in accelerometer registers:
  if( hardware_->read( this->getAccelAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), mode_change_accel_reg, &modechange_accel_, 1 ) < 0 )
  {
    ROS_ERROR("bmc050::readRegistersAccel(): MODECHANGE_ACCEL read failed.");
    return false;
  } 

  if( hardware_->read( this->getAccelAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), set_filter_reg, &set_filter_, 1 ) < 0 )
  {
    ROS_ERROR("bmc050::readRegistersAccel(): SET_FILTER read failed.");
    return false;
  } 

  //ROS_INFO("ACC: 0x11.   Val 0x%x",modechange_accel_);
  //ROS_INFO("ACC: 0x13.   Val 0x%x",set_filter_);

  return true;
}


/**********************************************************************/

/**********************************************************************/
bool BMC050::readRegistersCompass()
{
  // Read RATE_COMPASS and SOFTRESET registers into our copies
  // (which are modechange_accel_, modechange_, and softreset_)
  // so that we can set bits by writing back the whole register.

  // reading depends on the protocol:
  uint8_t mode_reg;
  uint8_t reset_reg;

  switch( protocol_ )
  {
  case I2C:
    // normal registers
    mode_reg  = RATE_COMPASS;
    reset_reg = SOFTRESET_COMPASS_REG;
    break;
  case SPI:
    // registers with the FLAG_R inserted
    mode_reg  = FLAG_R | RATE_COMPASS;
    reset_reg = FLAG_R | SOFTRESET_COMPASS_REG;
    break;
  default:
    ROS_ERROR("bmc050::readRegistersCompass(): invalid protocol.");
    return false;
  }
   
  // copy RATE_COMPASS located in compass registers:
  if( hardware_->read( this->getCompassAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), mode_reg, &rate_compass_, 1 ) < 0 )
  {
    ROS_ERROR("bmc050::readRegistersCompass(): RATE_COMPASS read failed.");
    return false;
  }

  // copy SOFTRESET located in compass registers:
  if( hardware_->read( this->getCompassAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), reset_reg, &softreset_, 1 ) < 0 )
  {
    ROS_ERROR("bmc050::readRegistersCompass(): SOFTRESET read failed.");
    return false;
  }
  //ROS_INFO("COMPASS: 0x4C.   Val 0x%x",rate_compass_);
  //ROS_INFO("COMPASS: 0x4B.   Val 0x%x",softreset_);
  return true;
}


/**********************************************************************/
/*INPUT: none directly. (indirectly) accel_range_, which was set 
 *     in the parameters.
 *OUTPUT: bool       -- whether or not it was successful.
 * This method performs two tasks:
 *   1. changes the accel_range to the accel_range set in the parameters.
 *   2. changes the class's accel_sensitivity, so that the output
 *    values will be multiplied correctly to produce correct values. 
 */
/**********************************************************************/
bool BMC050::changeAccelRange()
{
  // first change sensitivity based on current range: 
  switch( this->getAccelRange() )
  {
  case RANGE_2: // ±2 [g]
    accel_sensitivity_ =  0.00391; // units: [g/LSB]
    break;
  case RANGE_4: // ±4 [g]
    accel_sensitivity_ = 0.00781;  // units: [g/LSB]
    break;
  case RANGE_8: // ±8 [g]
    accel_sensitivity_ = 0.01562; // units: [g/LSB]
    break;
  case RANGE_16: // ±16 [g]
    accel_sensitivity_ = 0.03125;  // units: [g/LSB]
    break;
  default:
    ROS_ERROR("BMC050::changeAccelRange(): invalid accel range.");
    return false;
  }
  
  uint8_t local_range;
  // Read register with accel range settings for a local copy:
  if( this->readRegAccel( ADDRESS_ACCEL_RANGE, &local_range ) == false )
    return false;
  
  ROS_INFO("AccelRange bits before: %d.  Default: %d", (local_range & (0x0F)), 3); // Defaults on page 50
  
  // add our command to change range:
  local_range &= ~( 0x0F ); // clear old range value. Mask: b11110000
  local_range |= accel_range_; // insert new range value.
 
  // write the adjusted register value back to the sensor's register:
  if( this->writeToAccelReg( ADDRESS_ACCEL_RANGE, local_range ) == false )
    return false;
  
  // read back that register to make sure that changes worked:
  if( this->readRegAccel( ADDRESS_ACCEL_RANGE, &local_range ) == false)
    return false;
  
  // Compare register values to what we expect:
  uint8_t expected_reg = accel_range_;
  uint8_t actual_reg = local_range;
 
  // #ifdef DEBUG                       
  ROS_INFO("AccelRange bits after:  %d.  Expected: %d", actual_reg, expected_reg);
  // #endif
  
  if( expected_reg != actual_reg )
    return false;
  
  return true;
}


/**********************************************************************/
/*INPUT:  bool request  -- whether or not we want to filter data
 *OUTPUT: bool     -- whether or not changing the filter setting 
 *             worked.
 * This method performs two tasks:
 *  1. writes the user-defined bandwidth to sensor registers, if it 
 *   differs from the default.
 *  2. writes 1 or 0 to the DATA_HIGH_BW bit (without changing any other
 *   bits) to the appropriate SET_FILTER register: 0x13. 
 */
/**********************************************************************/
bool BMC050::filterData( bool request )
{
  uint8_t local_bandwidth;
  // Read register with accel range settings for a local copy:
  if( this->readRegAccel( SET_FILTER, &local_bandwidth ) == false )
    return false;
  
  // #ifdef DEBUG                            
  ROS_INFO("Accelerometer bandwidth bits before: %d.  Default: %d",  ((local_bandwidth >> DATA_HIGH_BW) & 0x01), 0); // Defaults on page 50.
  // #endif 

  // add our command to apply filter: (see datasheet page 21)
  if( request == true )
    local_bandwidth &= ~( 1 << DATA_HIGH_BW ); // filtered: write 0
  else
    local_bandwidth |= ( 1 << DATA_HIGH_BW );  // unfiltered: write 1
  
  // write the adjusted register value back to the sensor's register:
  if( this->writeToAccelReg( SET_FILTER, local_bandwidth ) == false )
    return false;
  
  // read back that register to make sure that changes worked:
  if( this->readRegAccel( SET_FILTER, &local_bandwidth ) == false )
    return false;
  
  // Compare register values to what we expect:
  uint8_t expected_reg = !accel_is_filtered_;
  uint8_t actual_reg = ( (local_bandwidth >> DATA_HIGH_BW) & 0x01 );
 
  // #ifdef DEBUG                            
  ROS_INFO("Accelerometer bandwidth bits after:  %d.  Expected: %d", actual_reg, expected_reg);
  // #endif
  
  if( expected_reg != actual_reg )
    return false;
  
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMC050::changeAccelBandwidth()
{
  uint8_t local_bw_reg;
  // read reg for a local copy:
  if( this->readRegAccel( 0x10, &local_bw_reg ) == false )
    return false;
 
  ROS_INFO("Accelerometer bandwidth setting before: %d.  Default:  %d", (local_bw_reg & 0x1F), 0x1F); // Defaults on page 50.
 
  // apply new bandwidth value:
  local_bw_reg &= ~( 0x1F ); //Mask: 1110000
  local_bw_reg |= bw_reg_;
  // write back reg with new bandwidth value:
  if( this->writeToAccelReg( 0x10, local_bw_reg ) == false )
    return false;
  // verify that we wrote back the correct value:
  uint8_t actual_reg;
  if( this->readRegAccel( 0x10, &actual_reg) == false )
    return false;
 
  uint8_t actual_bw = actual_reg & 0x1F; // mask off unrelated bits.
  uint8_t expected_bw = bw_reg_;
 
  if( expected_bw != actual_bw )
  {
    ROS_ERROR("Accelerometer bandwidth setting after: %d.  Expected: %d", actual_bw, bw_reg_);
    ROS_ERROR("BMC050::changeAccelBandwidth(): failed.");
    return false;
  }
  else
    ROS_INFO("Accelerometer bandwidth setting after: %d.  Expected: %d", actual_bw, bw_reg_);
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMC050::changeNumRepetitionsXY()
{
  uint8_t reg_before;
 
  this->readRegCompass( ADDRESS_COMPASS_REPXY, &reg_before );
  ROS_INFO("Compass REPXY before: %d  Default: %d  ", reg_before, 0x00 ); // on datasheet page 69

  // write class-defined value to register:
  this->writeToCompassReg( ADDRESS_COMPASS_REPXY, repsXY_ );
 
  uint8_t actual;
 
  this->readRegCompass( ADDRESS_COMPASS_REPXY, &actual );
  // verify that repsXY_ was written
  if( repsXY_ != actual )
  {
    ROS_ERROR("Compass REPXY after: %d  Expected: %d ", actual, repsXY_); 
    return false;
  }
  // if it all works: 
  ROS_INFO("Compass REPXY after: %d  Expected: %d ", actual, repsXY_); 
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMC050::changeNumRepetitionsZ()
{
  uint8_t reg_before;
 
  this->readRegCompass( ADDRESS_COMPASS_REPZ, &reg_before );
  ROS_INFO( "Compass REPZ before: %d  Default: %d", reg_before, 0x00 ); // on datasheet page 69 

  // write class-defined value to register:
  this->writeToCompassReg( ADDRESS_COMPASS_REPZ, repsZ_ );
 
  uint8_t actual;
 
  this->readRegCompass( ADDRESS_COMPASS_REPZ, &actual );
  // verify that repsXY_ was written
  if( repsZ_ != actual )
  {
    ROS_ERROR("Compass REPZ after: %d  Expected: %d ", actual, repsZ_); 
    return false;
  }

  ROS_INFO("Compass REPZ after: %d  Expected: %d", actual, repsZ_); 
  return true;
}

  
/**********************************************************************/
/**********************************************************************/
void BMC050::simpleCalibrationAccel()
{
  std::string axes[3] = { "X", "Y", "Z" };
  uint8_t offset_target_val[3] = { 0, 0, 1 }; // These bits go into register 0x37 to define the current xyz-alignment of the sensor with gravity (in [g]s)
  uint8_t offset_target[3] = { offset_target_x, offset_target_y, offset_target_z };
  uint8_t axis_bit[3] = { hp_x_en, hp_y_en, hp_z_en };
  uint8_t local_offset_reg;
  uint8_t local_calibration_status;               
 
  // Apply initial setup parameters:
 
  // Print initial register settings (filtered and unfiltered):
  this->printOffsets();

  // Query User
  std::cout << "Place the sensor flat and upright on a level surface." << std::endl
	    << "Warning: Do not move the sensor during calibration." << std::endl
	    <<"Press [y/n] to continue/cancel." << std::endl;
  std::string reply;
  std::getline( std::cin, reply );
  if ((reply != "Y") && (reply != "y"))
  {
    ROS_INFO("Calibration cancelled by user.");
    return;
  }  
 
  // Begin calibration on each axis sequentially:
  std::cout << "Beginning Simple Calibration routine:" << std::endl << std::endl;
  for (int i=0;i<3;i++)
  {
    std::cout << "Beginning Axis: " << axes[i] << std::endl;
    // First Write offset_target value to that axis to define its orientation:
    if( this->readRegAccel( ADDRESS_OFFSET_TARGET, &local_offset_reg ) == false )
    {
      std::cout<< "Error occured" << std::endl;
      return;
    }
    local_offset_reg &= (0x03 << offset_target[i]); // mask off old bitflags;
    local_offset_reg |= (offset_target_val[i] << offset_target[i]); // insert new bitflags;
  
    if( this->writeToAccelReg( ADDRESS_OFFSET_TARGET, local_offset_reg ) == false )
    {
      std::cout<< "Error occured" << std::endl;
      return;
    }
  
    // Next Trigger Calibration for that axis:
    if( this->readRegAccel( ADDRESS_ENABLE_CALIBRATION, &local_calibration_status ) == false )
    {
      std::cout<< "Error occured" << std::endl;
      return;
    }
    local_calibration_status &= ~0x07; // mask off current calibration-enable flags;
    local_calibration_status |= (1 << axis_bit[i]);
  
    if( this->writeToAccelReg( ADDRESS_ENABLE_CALIBRATION, local_calibration_status ) == false )
    {
      std::cout<< "Error occured" << std::endl;
      return;
    }
    std::cout << "Calibrating..." << std::endl;
  
    sleep( 3 ); // during this time, the sensor is updating the offset value every 8 samples (at 2 [kHz]) and adjusting the offset register in small increments.
    std::cout << "Done Calibrating Axis: " << axes[i] << std::endl << std::endl;
  
  }
  std::cout << "ACCELEROMETER CALIBRATION COMPLETE" << std::endl;
  // ask about permanently writing values to EEPROM:
 
  // Print final register settings (filtered and unfiltered):
  this->printOffsets(); 
}


/**********************************************************************/
// read both sensors separately
/**********************************************************************/
bool BMC050::takeMeasurement()
{
  // read the accel data and compass data and update their values:
 
  // We must perform two reads, since they are separate devices:
  this->getAccelData();
  this->getCompassData();
  return true;
}


/**********************************************************************/

/**********************************************************************/
bool BMC050::getAccelData()
{
  // gets raw data and updates 4 values: AccelX_, AccelY_, AccelZ_, Temperature_;
  uint8_t Data[7];
  uint8_t address;
  // The sensor treats reads differently depending on the protocol: 
  switch( this->getProtocol() )
  {
  case I2C:
    address = ADDRESS_ACCLXYZ;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_ACCLXYZ);
    break;
  default:
    ROS_ERROR("bmc050::getAccelData(): cannot read from this protocol.");
    return false;
  }
  
  if( hardware_->read( this->getAccelAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 7 ) < 0 )
  {
    ROS_ERROR("bmc050::getAccelData(): Error reading from hardware interface!");
    return false;
  }
  
  AccelX_ = this->getAccelSensitivity() * int( ( ((int16_t)(Data[1] << 8)) | ( ((int16_t)Data[0]) ) ) >> 6 );
  AccelY_ = this->getAccelSensitivity() * int( ( ((int16_t)(Data[3] << 8)) | ( ((int16_t)Data[2]) ) ) >> 6 );
  AccelZ_ = this->getAccelSensitivity() * int( ( ((int16_t)(Data[5] << 8)) | ( ((int16_t)Data[4]) ) ) >> 6 );
 
  Temperature_ = ( TempSlope_ *(int8_t)Data[6] ) + 24;       
       
  return true;
}


double BMC050::getAccelX()
{
  // gets raw data and updates : AccelX_
  uint8_t Data[2];
  uint8_t address;
  // The sensor treats reads differently depending on the protocol: 
  switch( this->getProtocol() )
  {
  case I2C:
    address = ADDRESS_ACCLXYZ;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_ACCLXYZ);
    break;
  default:
    ROS_ERROR("BMC050::getAccelX(): cannot read from this protocol.");
    return false;
  }
  // perform the read:  
  if( hardware_->read( this->getAccelAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getAccelX(): Error reading from I2C interface!");
    return false;
  }    
              
  AccelX_ = this->getAccelSensitivity() * int( ( ((int16_t)(Data[1] << 8)) | ( ((int16_t)Data[0]) ) ) >> 6 );
  return AccelX_;
}
 
 
/**********************************************************************/

/**********************************************************************/
double BMC050::getAccelY()
{
  // gets raw data and updates : AccelY_
  uint8_t Data[2];
  uint8_t address;
  // The sensor treats reads differently depending on the protocol: 
  switch( this->getProtocol() )
  {
  case I2C:
    address = ADDRESS_ACC_Y_LSB;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_ACC_Y_LSB);
    break;
  default:
    ROS_ERROR("BMC050::getAccelY(): cannot read from this protocol.");
    return false;
  }

  if( hardware_->read( this->getAccelAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getAccelY(): Error reading from I2C interface!");
    return false;
  }
              
  AccelY_ = this->getAccelSensitivity() * int( ( ((int16_t)(Data[1] << 8)) | ( ((int16_t)Data[0]) ) ) >> 6 );
  return AccelY_;
}


/**********************************************************************/

/**********************************************************************/
double BMC050::getAccelZ()
{
  // gets raw data and updates : AccelZ_
  uint8_t Data[2];
  uint8_t address;
  // The sensor treats reads differently depending on the protocol: 
  switch( this->getProtocol() )
  {
  case I2C:
    address = ADDRESS_ACC_Z_LSB;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_ACC_Z_LSB);
    break;
  default:
    ROS_ERROR("BMC050::getAccelZ(): cannot read from this protocol.");
    return false;
  }

  if( hardware_->read( this->getAccelAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getAccelZ(): Error reading from I2C interface!");
    return false;
  }
              
  AccelZ_ = this->getAccelSensitivity() * int( ( ((int16_t)(Data[1] << 8)) | ( ((int16_t)Data[0]) ) ) >> 6 );
  return AccelZ_;
}


/**********************************************************************/

/**********************************************************************/
double BMC050::getTemperature()
{
  // gets raw data and updates : Temperature_
  uint8_t Data[2];
  uint8_t address;
  // The sensor treats reads differently depending on the protocol: 
  switch( this->getProtocol() )
  {
  case I2C:
    address = ADDRESS_TEMPERATURE;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_TEMPERATURE);
    break;
  default:
    ROS_ERROR("BMC050::getTemperature(): cannot read from this protocol.");
    return false;
  }

  if( hardware_->read( this->getAccelAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getTemperature(): Error reading from I2C interface!");
    return false;
  }
 
  Temperature_ = ( TempSlope_ *(int8_t)Data[0] ) + 24;       
  return Temperature_;
}


/**********************************************************************/

/**********************************************************************/
bool BMC050::getCompassData()
{
  // gets raw data and updates 4 values: CompassX_, CompassY_, CompassZ_, RHall_;
  uint8_t Data[8];
  uint8_t address;
  // The sensor treats reads differently depending on the protocol: 
  switch( this->getProtocol() )
  {
  case I2C:
    address = ADDRESS_COMPASS_XYZ;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_COMPASS_XYZ);
    break;
  default:
    ROS_ERROR("BMC050::getCompassData(): cannot read from this protocol.");
    return false;
  }
  
  if( hardware_->read( this->getCompassAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 8 ) < 0 )
  {
    ROS_ERROR("BMC050::getCompassData(): Error reading from hardware interface!");
    return false;
  }          
 
  CompassX_ = SensitivityXY_ * ( (int16_t)( (int16_t)Data[0] | ((int16_t)Data[1]) << 8 ) / 8);
  CompassY_ = SensitivityXY_ * ( (int16_t)( (int16_t)Data[2] | ((int16_t)Data[3]) << 8 ) / 8);
  CompassZ_ = SensitivityZ_ *  ( (int16_t)( (int16_t)Data[4] | ((int16_t)Data[5]) << 8 ) / 2);
  RHall_ = (uint16_t)((uint16_t)Data[6] | ((uint16_t)Data[7] << 8) >> 2 );
  return true;
}


/**********************************************************************/

/**********************************************************************/
double BMC050::getCompassX()
{
  // gets raw data and updates CompassX_;
  uint8_t Data[2];
  uint8_t address;
 
  // reading depends on the Protocol! 
  switch( this->getProtocol() )
  {// The sensor treats reads differently depending on the protocol:
  case I2C:
    address = ADDRESS_COMPASS_X_LSB;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_COMPASS_X_LSB);
    break;
  default:
    ROS_ERROR("BMC050::getCompassX(): cannot read from this protocol.");
    return false;
  }            
  if( hardware_->read( this->getCompassAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getCompassX(): Error reading from hardware interface!");
    return false;
  }          

  CompassX_ = SensitivityXY_ * ( (int16_t)( (uint16_t)Data[0] | ((uint16_t)Data[1]) << 8 ) / 8);
  return CompassX_; 
}



/**********************************************************************/

/**********************************************************************/
double BMC050::getCompassY()
{
  // gets raw data and updates CompassY_;
  uint8_t Data[2];
  uint8_t address;

  // reading depends on the Protocol! 
  switch( this->getProtocol() )
  {// The sensor treats reads differently depending on the protocol:
  case I2C:
    address = ADDRESS_COMPASS_Y_LSB;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_COMPASS_Y_LSB);
    break;
  default:
    ROS_ERROR("BMC050::getCompassY(): cannot read from this protocol.");
    return false;
  }            
  if( hardware_->read( this->getCompassAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getCompassY(): Error reading from hardware interface!");
    return false;
  }          

  CompassY_ = SensitivityXY_ * ( (int16_t)( (uint16_t)Data[0] | ((uint16_t)Data[1]) << 8 ) / 8);
  return CompassY_;
}



/**********************************************************************/

/**********************************************************************/
double BMC050::getCompassZ()
{
  // gets raw data and updates CompassZ_;
  uint8_t Data[2];
  uint8_t address;

  // reading depends on the Protocol! 
  switch( this->getProtocol() )
  {// The sensor treats reads differently depending on the protocol:
  case I2C:
    address = ADDRESS_COMPASS_Z_LSB;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_COMPASS_Z_LSB);
    break;
  default:
    ROS_ERROR("BMC050::getCompassZ(): cannot read from this protocol.");
    return false;
  }            
  if( hardware_->read( this->getCompassAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getCompassZ(): Error reading from hardware interface!");
    return false;
  }          
      
  CompassZ_ = SensitivityZ_ * ( (int16_t)( (uint16_t)Data[0] | ((uint16_t)Data[1]) << 8 ) / 2);
  return CompassZ_;
}


/**********************************************************************/

/**********************************************************************/
uint16_t BMC050::getRHall()
{
  // gets raw data and updates RHall_;
  uint8_t Data[2];
  uint8_t address;

  // reading depends on the Protocol! 
  switch( this->getProtocol() )
  {// The sensor treats reads differently depending on the protocol:
  case I2C:
    address = ADDRESS_RHALL;
    break;
  case SPI:
    address = (FLAG_R | ADDRESS_RHALL);
    break;
  default:
    ROS_ERROR("BMC050::getRHall(): cannot read from this protocol.");
    return false;
  }            
  if( hardware_->read( this->getCompassAddress(), this->getProtocol(), this->getFrequency(), this->getFlags(), address, Data, 2 ) < 0 )
  {
    ROS_ERROR("BMC050::getRHall(): Error reading from hardware interface!");
    return false;
  }          

  RHall_ = (uint16_t)((uint16_t)Data[0] | ((uint16_t)Data[1] << 8) >> 2 );
  return RHall_; 
}


/**********************************************************************/

/**********************************************************************/
double BMC050::getUncompensatedYaw()
{
  // assumes you've already taken a measurement.
  return atan2( CompassX_, CompassY_ ) * ( 180.0 / M_PI );
}


/**********************************************************************/

/**********************************************************************/
uint8_t BMC050::getDeviceAddress()
{
  // The bmc050 cannot identify itself generically since it has two device addresses.
  ROS_ERROR("BMC050::getDeviceAddress(): Invalid request.  Accelerometer and Compass have separate device addresses.");
  return 0;
}


/**********************************************************************/

/**********************************************************************/
bool BMC050::enterLowPowerModeAccel()
{
  switch( accel_mode_ )
  {
  case accel_low_power_:
    ROS_WARN("BMC050::enterLowPowerModeAccel(): sensor already in low-power mode.");
    break;
    // normal and low-power modes enter suspend mode in the same way:
  case accel_normal_: {}
  case accel_suspend_:
    // write 1 to the suspend bit in accel register 0x11:
    // prepare our copy of the register:
    modechange_accel_ = ( (1 << LOWPOWER_EN) | modechange_accel_ ) & ~(1 << SUSPEND);
    // write our copy of the register:
    if( this->writeToAccelReg(MODECHANGE_ACCEL, modechange_accel_ ) == false )
    {
      ROS_ERROR("BMC050::enterLowPowerModeAccel(): write failed.");
      // retreive the sensor copy of the 0x11 register:
      this->readRegAccel( MODECHANGE_ACCEL, &modechange_accel_ );
      return false;
    }
    break;
  default:
    ROS_ERROR("BMC050::enterLowPowerModeAccel(): failed.");
    return false;
  }
  // set new mode:
  accel_mode_ = accel_low_power_;
  return true;
}


/**********************************************************************/
bool BMC050::enterSuspendModeAccel()
{
  switch( accel_mode_ )
  {
  case accel_suspend_:
    ROS_INFO("BMC050::enterSuspendModeAccel(): sensor already in suspend mode.");
    break;
    // normal and low-power modes enter suspend mode in the same way:
  case accel_normal_: {}
  case accel_low_power_:
    // write 1 to the suspend bit in accel register 0x11:
    // prepare our copy of the register:
    modechange_accel_ = ( (1 << SUSPEND) | modechange_accel_ ) & ~(1 << LOWPOWER_EN);
    // write our copy of the register:
    if( this->writeToAccelReg( MODECHANGE_ACCEL, modechange_accel_ ) == false )
    {
      ROS_ERROR("BMC050::enterSuspendModeAccel(): write failed.");
      // retreive the sensor copy of the 0x11 register:
      this->readRegAccel( MODECHANGE_ACCEL, &modechange_accel_ );
      return false;
    }
    break;
  default:
    ROS_ERROR("BMC050::enterSuspendModeAccel(): failed.");
    return false;
  }
  // set new mode:
  accel_mode_ = accel_suspend_;
  return true;
}


/**********************************************************************/
bool BMC050::enterNormalModeAccel()
{
  switch( accel_mode_ )
  {
  case accel_normal_:
    ROS_WARN("BMC050::enterNormalModeAccel(): sensor already in normal mode.");
    break;
    // suspend and low-power modes enter normal mode in the same way:
  case accel_suspend_: {}
  case accel_low_power_:
    // Clear both SUSPEND bit and LOWPOWER_EN bit in accel reg 0x11:
    // prepare our copy of the register:
    modechange_accel_ &= ~( (1 << SUSPEND) | (1 << LOWPOWER_EN) );
    // write our copy of the register:
    if( this->writeToAccelReg( MODECHANGE_ACCEL, modechange_accel_ ) == false )
    {
      ROS_ERROR("BMC050::enterNormalModeAccel(): write failed.");
      // retreive the sensor copy of the 0x11 register:
      this->readRegAccel( MODECHANGE_ACCEL, &modechange_accel_ );
      return false;
    }
    usleep( 800 ); // mandatory delay.  See datasheet page 8.
    break;
  default:
    ROS_ERROR("BMC050::enterNormalModeAccel(): failed.");
    return false;
  }
  // set new mode:
  accel_mode_ = accel_normal_;
  return true;
}


/**********************************************************************/
bool BMC050::enterSuspendModeCompass()
{
  // depends on the mode the sensor is currently in:
  switch( compass_mode_ )
  {
  case compass_suspend_:
    ROS_WARN("BMC050::enterSuspendModeCompass(): device already in Suspend Mode.");
    break;
  case compass_forced_:
    // sensor must enterSleepMode first; then enterSuspendMode from there:
    this->enterSleepModeCompass();  // don't break;
    // sleep_ and normal_ mode enterSuspendMode in the exact same way:
  case compass_sleep_: {}
  case compass_normal_: {}
    // adjust bitflags in our copy of the register:
    softreset_ &= (0 << POWER_CONTROL_BIT);
    // write our copy of the register:
    if( this->writeToCompassReg( SOFTRESET_COMPASS_REG, softreset_ ) == false )
    {
      ROS_ERROR("BMC050::enterSuspendModeCompass(): failed to write to registers.");
      // if unsuccessful, get an old copy of the register:
      this->readRegCompass( SOFTRESET_COMPASS_REG, &softreset_ );
      return false;
    }
    break;
  default:
    ROS_ERROR( "BMC050::enterSuspendModeCompass(): failed to enter Suspend Mode." );
    return false;
  }
  // adjust current mode:
  compass_mode_ = compass_suspend_;
  return true;
}


/**********************************************************************/
bool BMC050::enterSleepModeCompass()
{
  // depends on what mode the sensor is currently in:
  switch( compass_mode_ )
  {
  case compass_normal_: {}
  case compass_forced_:
    // adjust flags:
    rate_compass_ |= (SET_OPMODE_SLEEP << OPMODE); // shift b00000011 left by one bit and set those bits
    // set OPMODE bits here
    if( this->writeToCompassReg( RATE_COMPASS, rate_compass_ ) == false )
    {
      ROS_ERROR("BMC050::enterSleepModeCompass(): failed.");
      // revert flags:
      this->readRegCompass( RATE_COMPASS, &rate_compass_ );
      return false;
    }
    break;
  case compass_suspend_:
    // adjust flags
    softreset_ = (1 << POWER_CONTROL_BIT);
    // set new register value here
    if( this->writeToCompassReg( SOFTRESET_COMPASS_REG, softreset_ ) == false )
    {
      ROS_ERROR("BMC050::enterSleepModeCompass(): writing to registers failed.");
      // revert flags:
      this->readRegCompass( SOFTRESET_COMPASS_REG, &softreset_ );
      return false;
    }
    usleep( 3000 ); // mandatory delay --- see datasheet page 10
    break;
  case compass_sleep_:
    ROS_WARN("BMC050::enterSleepModeCompass():Device already in sleep mode.");
    break;
  default:
    ROS_ERROR("BMC050::enterSleepModeCompass(): failed.");
    return false;
  }
  // set new mode:
  compass_mode_ = compass_sleep_; 
  return true;
}


/**********************************************************************/
bool BMC050::enterNormalModeCompass()
{
  // depends on what mode we're already in:
  switch( compass_mode_ )
  {
    // suspend_ and forced_ mode must first enter sleep mode:
  case compass_suspend_: {}
  case compass_forced_: 
    // first enter sleep mode:
    this->enterSleepModeCompass(); // don't break;
    // then the sensor can directly enter normal mode:
  case compass_sleep_:
    // adjust flags
    rate_compass_ &= ~(SET_OPMODE_SLEEP << OPMODE);  // set flags;
    // set new register value here
    if( this->writeToCompassReg( RATE_COMPASS, rate_compass_ ) == false )
    {
      ROS_ERROR("BMC050::enterNormalModeCompass(): failed to write to registers.");
      // revert flags to sensor's flags:
      this->readRegCompass( RATE_COMPASS, &rate_compass_ );
      return false;
    } 
    break;
  case compass_normal_:
    // done!
    ROS_WARN("BMC050::enterNormalModeCompass(): sensor already in normal mode.");
    break;
  default:
    ROS_ERROR("BMC050::enterNormalModeCompass(): error entering Normal Mode.");
    return false;
  }
  // set new mode:
  compass_mode_ = compass_normal_;
  return true;
}


/**********************************************************************/
bool BMC050::enterForcedModeCompass()
{ 
  // depends on the mode the sensor is currently in:
  switch( compass_mode_ )
  {
  case compass_forced_:
    ROS_WARN("BMC050::enterForcedModeCompass(): device already in forced mode.");
    break;
    // can only directly be entered from sleepmode:
  case compass_suspend_: {}
  case compass_normal_: {}
    this->enterSleepModeCompass();  // don't break;
  case compass_sleep_:
    // adjust flags
    rate_compass_ &= ~(SET_OPMODE_SLEEP << OPMODE);  // clear flags;
    rate_compass_ |= (SET_OPMODE_FORCED << OPMODE ); // set new flags;
    // set new register value here
    if( this->writeToCompassReg( RATE_COMPASS, rate_compass_ ) == false )
    {
      ROS_ERROR("BMC050::enterForcedModeCompass(): failed to write to registers.");
      // revert flags to sensor's flags:
      this->readRegCompass( RATE_COMPASS, &rate_compass_ );
      return false;
    }
    break;
  default:
    ROS_ERROR("BMC050::enterForcedModeCompass(): failed.");
    return false;
  }
  compass_mode_ = compass_forced_;
  return true;
}


/**********************************************************************/
// reads a register and returns its value to the second argument
/**********************************************************************/
bool BMC050::readRegAccel( uint8_t reg, uint8_t* value )
{
  // Reading depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->read( this->getAccelAddress(), I2C, this->getFrequency(), this->getFlags(), reg, value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMC050::readRegAccel(): Error reading register via I2C!");
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG.
    if( hardware_->read( this->getAccelAddress(), SPI, this->getFrequency(), this->getFlags(), (1 << SPI_READ_FLAG) | reg, value, 1 ) < 0 )
    {
      ROS_ERROR("BMC050::readRegAccel(): Error reading register via SPI!");
      return false;
    } 
    break;
  default:
    ROS_ERROR("BMC050::readRegAccel(...): invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
// reads a register and returns its value.  (for testing)
/**********************************************************************/
bool BMC050::readRegCompass( uint8_t reg, uint8_t* value )
{
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->read( this->getCompassAddress(), I2C, this->getFrequency(), this->getFlags(), reg, value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMC050::readRegCompass(): Error reading register via I2C!");
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG, although, technically it's already there, since it's zero.
    if (hardware_->read( this->getCompassAddress(), SPI, this->getFrequency(), this->getFlags(), (1 << SPI_READ_FLAG) | reg, value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMC050::readRegCompass(): Error reading register via SPI!");
      return false;
    } 
    break;
  default:
    ROS_ERROR("BMC050::readRegistersCompass(...): invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
// writes a byte to a register in the Accelerometer
/**********************************************************************/
bool BMC050::writeToAccelReg( uint8_t reg, uint8_t value )
{
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->write( this->getAccelAddress(), I2C, this->getFrequency(), this->getFlags(), (uint8_t)reg, (uint8_t*)&value, 1 ) < 0 )
    {
      ROS_ERROR("BMC050::writeToAccelReg(): Error writing to register via I2C!");
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_WRITE_FLAG, although, technically it's already there, since it's zero.
    if( hardware_->write( this->getAccelAddress(), SPI, this->getFrequency(), this->getFlags(), (uint8_t) (~(1 << SPI_WRITE_FLAG)&reg), (uint8_t*)&value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMC050::writeToAccelReg(): Error writing to register via SPI!");
      return false;
    } 
    break;
  default:
    ROS_ERROR("BMC050::writeToAccelReg(...): invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
// writes a byte to a register in the compass.
/**********************************************************************/
bool BMC050::writeToCompassReg( uint8_t reg, uint8_t value )
{
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->write( this->getCompassAddress(), I2C, this->getFrequency(), this->getFlags(), (uint8_t)reg, (uint8_t*)&value, 1 ) < 0)
    {
      ROS_ERROR("BMC050::writeToCompassReg(): Error writing to register via I2C!");
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_WRITE_FLAG, although, technically it's already there, since it's zero.
    if( hardware_->write( this->getCompassAddress(), SPI, this->getFrequency(), this->getFlags(), (uint8_t) (~(1 << SPI_WRITE_FLAG)&reg), (uint8_t*)&value, 1 ) < 0 ) 
    {
      ROS_ERROR("BMC050::writeToCompassReg(): Error writing to register via SPI!");
      return false;
    } 
    break;
  default:
    ROS_ERROR("BMC050::writeToCompassReg(...): invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
// writes a byte to a compass register.  Reads it back to verify.
/**********************************************************************/
bool BMC050::writeToCompassRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected )
{
  this->writeToCompassReg( reg, value );
 
  uint8_t actual;
 
  this->readRegCompass( reg, &actual );
 
  if( expected != actual )
  {
    ROS_ERROR("BMC050::writeToCompassRegAndVerify(...): Compass Register: %d  actual: %d  expected: %d ", reg, actual, expected); 
    return false;
  }
 
  return true;
}


/**********************************************************************/
// writes a byte to an accel register.  Reads it back to verify.
/**********************************************************************/
bool BMC050::writeToAccelRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected )
{
  this->writeToAccelReg( reg, value );
 
  uint8_t actual;
 
  this->readRegAccel( reg, &actual );
 
  if( expected != actual )
  {
    ROS_ERROR("BMC050::writeToAccelRegAndVerify(...): Accelerometer Register: %d  actual: %d  expected: %d", reg, actual, expected); 
    return false;
  }

  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMC050::printOffsets()
{
  uint8_t reg_to_print;    
  std::string offset_reg_names[6] = { "offset_filt_x", "offset_filt_y", "offset_filt_z", "offset_unfilt_x", "offset_unfilt_y", "offset_unfilt_z" };
 
  for( uint8_t i = OFFSET_FILT_X; i <= OFFSET_UNFILT_Z; i++ )
  {
    if( this->readRegAccel( i, &reg_to_print ) == false )
      return false;

    std::cout << offset_reg_names[i - OFFSET_FILT_X] << ": " << (int)reg_to_print << std::endl;
  }
  return true;
}
