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
 
//\Author Joshua Vasquez and  Philip Roan, Robert Bosch LLC

#ifndef BMA180_H_
#define BMA180_H_

#include <cmath>  // for atan2
#include <unistd.h> // for the sleep function

#include <bosch_drivers_sensor_driver.hpp>
#include <bosch_drivers_hardware_interface.hpp>

#include "bma180_parameters.hpp"

using namespace bosch_drivers_common;

/**
 * \brief Driver the the BMA180 accelerometer.
 *
 * This class provides access to both the acceleration and temperature data.
 * Users can change the sensor's basic settings, such as bandwidth and range;
 * however, the user cannot change modes and cannot directly implement the
 * sensor's built-in slope-detection routines.
 */
class BMA180: public sensor_driver, public BMA180Parameters
{
public:
  // Constructor
  BMA180( bosch_hardware_interface* hw ); 
  // Destructor
  ~BMA180();
    
  // Initialize the hardware interface so it can communicate with the bma180:
  bool initialize();
   
  // Measurement Methods
  bool takeMeasurement();
  bool getAccelData();
  double getAccelX();
  double getAccelY();
  double getAccelZ();
  double getStaticPitch(); // returns the pitch.  Measuring while moving is a bad idea, for now.
  double getStaticRoll();  
   
  double getTemperature();
  
  // Preferance Adjustments
  bool EnableWriting(); // must be called to perform any writes.
  bool Calibrate();  // a software routine for "re-zeroing the output."
  bool CoarseCalibrate();
  bool TwoStepCoarseCalibrate();
  bool FullCalibration();
  bool softReset();
  bool DisableI2C(); // to get bug-free SPI readings.
  bool changeAccelRange(); 
  bool changeBandwidth();
  //bool togglePreCalOffsets(bool choice);
  bool setEnOffsetBit( uint8_t bit );
  
  //returns the way the sensor identifies itself to the hardware interface.
  uint8_t getDeviceAddress(); 

  // Inheretted Methods: (from bma180_parameters)
  //bool setProtocol(interface_protocol protocol);
  //bool setFrequency(int frequency);
  //bool setPin( uint8_t pin);       
  //bool setSlaveAddressBit(bool choice); // choose between two device addresses. 
  //bool setByteOrder(uint8_t value);
  //bool setSpiMode(uint8_t mode);
  //bool setRange(accel_range new_range);
  //double        getSensitivity(); // returns sensitivity
  //interface_protocol  getProtocol();
  //int         getFrequency();
  //int*         getFlags();
  //int         getPin();  // SPI, GPIO only
  //uint8_t       getSlaveAddressBit();

  /* ----- MEMBERS ----- */
  double AccelX_;
  double AccelY_;
  double AccelZ_;
  double Temperature_;
  double StaticPitch_;
  double StaticRoll_;  
  
  double TempSlope_;
  
  
protected:
  // BMA180 Register Definitions

  /**
   * \note an optional i2c address
   * \note used if SDO is connected to VSS
   */
  static const uint8_t SLAVE_ADDRESS0 = 0x40;

  /**
   * \note an optional i2c address
   * \note  if SDO is connected to VDDIO
   */
  static const uint8_t SLAVE_ADDRESS1 = 0x41; 

  // The Acceleration Data Registers
  static const uint8_t ADDRESS_ACCLXYZ     = 0x02;
  static const uint8_t ADDRESS_ACCLX_MSB   = 0x03;
  static const uint8_t ADDRESS_ACCLY_LSB   = 0x04;
  static const uint8_t ADDRESS_ACCLY_MSB   = 0x05;
  static const uint8_t ADDRESS_ACCLZ_LSB   = 0x06;
  static const uint8_t ADDRESS_ACCLZ_MSB   = 0x07;
  static const uint8_t ADDRESS_TEMPERATURE = 0x08;

  // Four registers for storing our own data
  static const uint8_t ADDRESS_EE_CD1 = 0x4C;
  static const uint8_t ADDRESS_EE_CD2 = 0x4D;
  static const uint8_t ADDRESS_CD1    = 0x2C;
  static const uint8_t ADDRESS_CD2    = 0x2D;

  static const uint8_t ADDRESS_VER         = 0x00;  
  static const uint8_t ADDRESS_STATUS_REG1 = 0x09;
  static const uint8_t ADDRESS_STATUS_REG2 = 0x0A;
  static const uint8_t ADDRESS_STATUS_REG3 = 0x0B;
  static const uint8_t ADDRESS_STATUS_REG4 = 0x0C;
  static const uint8_t ADDRESS_CTRLREG0    = 0x0D;
  static const uint8_t ADDRESS_CTRLREG1    = 0x0E;
  static const uint8_t ADDRESS_CTRLREG2    = 0x0F;
  static const uint8_t ADDRESS_CTRL_REG4   = 0x22;

  /// Calibration addresses in the EEPROM
  static const uint8_t ADDRESS_EE_OFFSET_Z    = 0x5A; 
  static const uint8_t ADDRESS_EE_OFFSET_Y    = 0x59;  
  static const uint8_t ADDRESS_EE_OFFSET_X    = 0x58;    
  static const uint8_t ADDRESS_EE_OFFSET_T    = 0x57;  
  static const uint8_t ADDRESS_EE_OFFSET_LSB2 = 0x56;  
  static const uint8_t ADDRESS_EE_OFFSET_LSB1 = 0x55;  
  static const uint8_t ADDRESS_EE_GAIN_Z      = 0x54;  
  static const uint8_t ADDRESS_EE_GAIN_Y      = 0x53;  
  static const uint8_t ADDRESS_EE_GAIN_X      = 0x52;  
  static const uint8_t ADDRESS_EE_GAIN_T      = 0x51;  

  /// Calibration addresses NOT in EEPROM. Deleted every power cycle.
  static const uint8_t ADDRESS_OFFSET_Z    = 0x3A;
  static const uint8_t ADDRESS_OFFSET_Y    = 0x39;
  static const uint8_t ADDRESS_OFFSET_X    = 0x38;
  static const uint8_t ADDRESS_OFFSET_T    = 0x37;
  static const uint8_t ADDRESS_OFFSET_LSB2 = 0x36;
  static const uint8_t ADDRESS_OFFSET_LSB1 = 0x35;
  static const uint8_t ADDRESS_GAIN_Z      = 0x34;
  static const uint8_t ADDRESS_GAIN_Y      = 0x33;
  static const uint8_t ADDRESS_GAIN_X      = 0x32;
  static const uint8_t ADDRESS_GAIN_T      = 0x31;

  /// INTEGRATED DIGITAL FILTER:
  static const uint8_t ADDRESS_BW_TCS = 0x20;

  /// Soft-Reset
  static const uint8_t ADDRESS_SOFTRESET = 0x10;
  static const uint8_t CMD_SOFTRESET     = 0xB6;

  /// Disable-I2C
  static const uint8_t ADDRESS_HIGH_DUR = 0x27;
  static const uint8_t CMD_DISABLE_I2C  = 0x01;

  /// (relevant) BITFLAGS for changing settings:

  // BitFlags: CTRL_REG0
  // Enable Writing
  static const uint8_t ee_w = 4; // 1 bit of data < 3 > 

  // BitFlags: ADDRESS_OFFSET_LSB1
  // Changing range
  static const uint8_t range = 1; // 3 bits of data  < 3:1 >

  // BitFlags: ADDRESS_BW_TCS
  // Changing Bandwidth
  static const uint8_t bw = 4;
 
  // BitFlags: CTRL_REG1
  // Enable pre-calibrated offset flags: see Datasheet
  static const uint8_t en_offset_x = 7;
  static const uint8_t en_offset_y = 6;
  static const uint8_t en_offset_z = 5;
 
  // BitFlags: ADDRESS_STATUS_REG1
  // Check if calibration has been completed:
  static const uint8_t offset_st_s = 1;
 
  // BitFlags: ADDRESS_CTRL_REG4
  // Begin a calibration routine:
  static const uint8_t offset_finetuning = 0; // 2 bits of data: < 1:0 >


  bool readReg( uint8_t reg, uint8_t* value );             
  bool writeToReg( uint8_t reg, uint8_t value );
  bool writeToRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected_value );
  bool readSensorData( uint8_t reg, uint8_t* value, uint8_t num_bytes );  
};

#endif // BMA180_H_
