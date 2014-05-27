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

#ifndef BMC050_H_
#define BMC050_H_

#include <cmath>
#include <unistd.h> // for the sleep function
#include <iostream> // for the calibration routine

// ROS debugging output
#include <ros/console.h>

#include <bosch_drivers_sensor_driver.hpp>
#include <bosch_drivers_hardware_interface.hpp>

#include "bmc050_parameters.hpp"

using namespace bosch_drivers_common;

/**
 * \brief      a bmc050 driver that implements the device's main 
 * capabilities.
 *
 * \details     This class provides access to the acceleration 
 * temperature, and Magnetic field data.  Routines have been written to 
 * change settings on both sensors, such as bandwidth and range.
 * Routines have also been written to change modes.
 * \note       Not all capabilities of the sensors have been 
 * implemented.  For example, the user cannot directly implement the 
 * accelerometer's built-in slope-detection routines.  However, 
 * these routines may be implemented using several of the existing 
 * methods.
 * \note     This class implements its methods through a generic
 * hardware interface.
 */
class BMC050: public sensor_driver, public BMC050Parameters
{

public:
 
 /*----- Bit Flags --------------------------------------------------- */
 // class copies of the sensor registers, so not to overwrite data 
 /**
  * \brief the following are the class values of several important 
  *     registers on both sensors.  Upon initialization, the sensor
  *     reads and stores values to these class members
  * \var uint8_t mode_change_accel_
  * \var uint8_t softreset_
  * \var uint8_t rate_compass_
  * \var uint8_t set_filter_
  */
  uint8_t modechange_accel_;         
  uint8_t softreset_;            
  uint8_t rate_compass_;       
  uint8_t set_filter_;    
 
  // POWERFLAGS
  static const uint8_t SUSPEND     = 0x07; // b7  
  static const uint8_t LOWPOWER_EN = 0x06; // b6
  static const uint8_t SLEEP_DUR   = 0x01; // b1 is the lsb
 
  // changing modes
  static const uint8_t SET_OPMODE_SLEEP  = 0x03;
  static const uint8_t SET_OPMODE_FORCED = 0x01;
  static const uint8_t OPMODE            = 0x01; // (b1 is the lsb)  
  static const uint8_t POWER_CONTROL_BIT = 0x00;

  // Specific commands
  static const uint8_t SOFTRESET_COMPASS_CMD = 0x82;
  static const uint8_t SOFTRESET_ACCEL_CMD   = 0xB6;
  static const uint8_t FLAG_R                = 0x80; // for SPI reads

  // BitFlags: BANDWIDTH REG
  static const uint8_t DATA_HIGH_BW = 7; // <7>
  // BitFlags: RATE_COMPASS
  static const uint8_t Data_Rate    = 3; // <5:3> in 0x4C
 
  // BitFlags: ADDRESS_ENABLE_CALIBRATION
  static const uint8_t hp_z_en = 2;
  static const uint8_t hp_y_en = 1;
  static const uint8_t hp_x_en = 0;

  // BitFlags: ADDRESS_OFFSET_TARGET register:
  static const uint8_t offset_target_x = 1; // <2:1>
  static const uint8_t offset_target_y = 3; // <4:3>
  static const uint8_t offset_target_z = 5; // <6:5>

  enum compass_mode
  {
    compass_suspend_,
    compass_sleep_,
    compass_normal_,
    compass_forced_
  };

  enum accel_mode
  {
    accel_suspend_,
    accel_normal_,
    accel_low_power_
  };

  // Constructor
  /**
   * \param  hw  A pointer to the hardware interface that the device is 
   *        connected to.
   */
  BMC050( bosch_hardware_interface* hw );

  // Destructor
  ~BMC050();
  /**
   * \brief  An invalid request.  It is necessary, since this driver
   *      inherits from the sensor_driver class. See the
   *      getAccelAddress() and getCompassAddress() methods in the 
   *      corresponding parameters class. 
   *
   * The user does not need to use this method.
   * \return 0 An invalid address, since each sensor has its own address. 
   */ 
  uint8_t getDeviceAddress(); 

  /**
   * \brief initializes the sensor with all user-defined
   *     preferences indicated in the parameters.
   *
   * \return a boolean indicating success
   */
  bool initialize();

  /**
   * \brief reads important accelerometer registers for a local copy. 
   *     This method is needed for changing between modes, which 
   *     determine values in multiple registers.
   *
   * \return a boolean indicating success.
   */
  bool readRegistersAccel();

  /**
   * \brief reads important compass registers for a local copy.
   *     This method is needed for changing between modes, which
   *     determine values in multiple registers.
   *
   * \return a boolean indicating success.
   */
  bool readRegistersCompass();

  /**
   * \brief perform two separate readings, one for each device.
   *
   * \return a boolean indicating success
   */
  bool takeMeasurement();

  /**
   * \brief  resets the accelerometer, putting it back into normal mode
   *      and clearing its image.
   *
   * \return  a boolean indicating whether or not the hardware sent the 
   *      reset command.
   */
  bool softResetAccel();

  /**
   * \brief  a method that changes the accelerometer's range by writing 
   *      to the appropriate sensor register.  The range is specified 
   *      by the setAccelRange(range) in the inheritted parameter's
   *      class.
   *
   * The range will be changed to the default range if the user
   *      has not specified a range.
   * \return  a boolean indicating success
   */
  bool changeAccelRange();         
 
  /**
   * \brief writes to the sensor's register in memory to choose whether or 
   *     not to filter the accelerometer data.
   *
   * \param request  a boolean indicating the user's choice of filtered
   *     (true) or unfiltered (false).
   * \note  The data will be set as unfiltered if the user has not 
   *     specified an alternate request.
   * \return a boolean indicating whether or not the sensor register has 
   *      adjusted to reflect the request.
   */ 
  bool filterData( bool request );    

  /**
   * \brief changes the accelerometer's bandwidth by writing to the 
   *     appropriate sensor register.  The bandwidth is specified by
   *     the setAccelBandwidth(bandwidth) method in the inheritted 
   *     parameters class.
   * \note  the bandwidth will be changed to the default bandwidth if the 
   *     user has not specified a bandwidth.
   * \return a boolean indicating whether or not the sensor register has
   *      adjusted to reflect the requested bandwidth.
   */ 
  bool changeAccelBandwidth();       

  /**
   * \brief  retrieves all available raw accelerometer data and stores 
   *      the calculated value to class variables: AccelX_, AccelY_, 
   *      AccelZ_, Temperature_.
   *
   * \return  a boolean indicating success.
   */
  bool getAccelData();
  
  /**
   * \brief  retrieves the raw accelerometer x-axis data and stores the
   *      calculated value to the class variable: AccelX_
   *
   * If the application requires more than just one axis of data,
   *      use the getAccelData() instead, as it is much more
   *      efficient.
   * \return  a boolean indicating success.
   */
  double getAccelX();

  /**
   * \brief  retrieves the raw accelerometer y-axis data and stores the
   *      calculated value to the class variable: AccelY_
   *
   * If the application requires more than just one axis of data,
   *      use the getAccelData() instead, as it is much more
   *      efficient.
   * \return  a boolean indicating success.
   */
  double getAccelY();

  /**
   * \brief  retrieves the raw accelerometer z-axis data and stores the
   *      calculated value to the class variable: AccelZ_
   * \note  if the application requires more than just one axis of data,
   *      use the getAccelData() instead, as it is much more
   *      efficient.
   * \return  a boolean indicating success.
   */
  double getAccelZ();

  /**
   * \brief  retrieves the raw accelerometer temperature data and 
   *      stores the calculated value to a class variable.
   * If the application requires more than just one axis of data,
   *      use the getAccelData() instead, as it is much more
   *      efficient.
   * \return  a boolean indicating success.
   */  
  double getTemperature();

  /**
   * \brief  calibrates the sensor assuming it is in a static, flat,
   *      level, face-up position (ON EARTH!).
   *
   * A calibration should be necessary only if the 
   *      accelerometer does not produce accurate measurements.  The 
   *      sensor should already come from the factory with
   *      well-defined pre-calibrated constants.
   * \note  This calibration does NOT permanently override the sensor's
   *      pre-calibrated constants. 
   */
  void simpleCalibrationAccel();

  // Compass: settings
  /**
   * \brief  resets the compass, putting it back into "suspend" mode.
   *
   * If this method is called after the initialize method, the
   *      user must put the compass into normal mode before data other
   *      settings can be changed.  Use: the enterNormalModeCompass()
   *      method.
   * \return  a boolean indicating whether or not the hardware has sent
   *      the reset command.
   */
  bool softResetCompass();

  /**
   * \brief  change the output-data rate of the compass to the value
   *      specified in the parameters.
   *
   * \note   if no value has been specified in the parameters, the value
   *      will be changed to the default value.
   * \note  this function is called when \a initialize() is called.
   * \return  a boolean indicating that the compass output-data rate has
   *      been changed on the compass.
   */
  bool changeCompassRate(); 

  /**
   * \brief  change the number of samples that the compass averages 
   *      for the X and Y-axis data before outputting a value.
   *
   * \return  a boolean indicating success.
   */
  bool changeNumRepetitionsXY();

  /**
   * \brief  change the number of samples that the compass averages 
   *      for the X and Y-axis data before outputting a value.
   *
   * \return  a boolean indicating success.
   */
  bool   changeNumRepetitionsZ();
 
  // Compass: data
  /**
   * \brief  take a full measurement from the compass sensor through a 
   *      generic hardware interface.
   *
   * \return  accel_sensitivity_
   */
  bool    getCompassData();

  double   getCompassX();
  double   getCompassY();
  double   getCompassZ();
  uint16_t  getRHall();
 
  /**
   * \brief a simple test method to get the orientation of the sensor 
   *     assuming that the sensor is perfectly flat.
   *
   * \return a double indicating the orientation in degrees.
   */
  double getUncompensatedYaw();
 
  // Modes and Settings (Power Management, Overall Behavior)
  bool enterLowPowerModeAccel();      //|
  bool enterSuspendModeAccel();       //|--> not yet implemented.
  bool enterNormalModeAccel();       //|
  //bool setBandwidth(accel_bandwidth bw); //|

  bool enterSuspendModeCompass(); 
  bool enterSleepModeCompass(); 
  bool enterNormalModeCompass();  
  bool enterForcedModeCompass();

  bool readRegAccel( uint8_t reg, uint8_t* value );                
  bool readRegCompass( uint8_t reg, uint8_t* value );                 
  bool writeToAccelReg( uint8_t reg, uint8_t value );
  bool writeToCompassReg( uint8_t reg, uint8_t value );
  bool writeToCompassRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected );
  bool writeToAccelRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected );
  bool printOffsets();

//private:
  // Data Registers
  static const uint8_t ADDRESS_ACCLXYZ   = 0x02;
  static const uint8_t ADDRESS_ACC_X_LSB = 0x02;
  static const uint8_t ADDRESS_ACC_X_MSB = 0x03;
  static const uint8_t ADDRESS_ACC_Y_LSB = 0x04;
  static const uint8_t ADDRESS_ACC_Y_MSB = 0x05;
  static const uint8_t ADDRESS_ACC_Z_LSB = 0x06;
  static const uint8_t ADDRESS_ACC_Z_MSB = 0x07;
 
  static const uint8_t ADDRESS_TEMPERATURE = 0x08;
 
  static const uint8_t ADDRESS_ENABLE_CALIBRATION = 0x36;
  static const uint8_t ADDRESS_OFFSET_TARGET      = 0x37;
  static const uint8_t OFFSET_FILT_X              = 0x38;
  static const uint8_t OFFSET_UNFILT_Z            = 0x3D;
 
  static const uint8_t ADDRESS_COMPASS_XYZ   = 0x42;
  static const uint8_t ADDRESS_COMPASS_X_LSB = 0x42;
  static const uint8_t ADDRESS_COMPASS_X_MSB = 0x43;
  static const uint8_t ADDRESS_COMPASS_Y_LSB = 0x44;
  static const uint8_t ADDRESS_COMPASS_Y_MSB = 0x45;
  static const uint8_t ADDRESS_COMPASS_Z_LSB = 0x46;
  static const uint8_t ADDRESS_COMPASS_Z_MSB = 0x47;
 
  static const uint8_t ADDRESS_RHALL     = 0x48;
  static const uint8_t ADDRESS_RHALL_LSB = 0x48;
  static const uint8_t ADDRESS_RHALL_MSB = 0x49;
 
  static const uint8_t ADDRESS_COMPASS_REPZ  = 0x51;
  static const uint8_t ADDRESS_COMPASS_REPXY = 0x52;
 
  static const uint8_t ADDRESS_ACCEL_RANGE   = 0x0F;

  // Settings Registers
  // Accel
  static const uint8_t MODECHANGE_ACCEL = 0x11;
  static const uint8_t SOFTRESET_ACCEL  = 0x14;
  static const uint8_t BANDWIDTH_REG    = 0x10;
  static const uint8_t SET_FILTER       = 0x13;
  // Compass
  static const uint8_t RATE_COMPASS     = 0x4C;
  static const uint8_t SOFTRESET_COMPASS_REG = 0x4B;


  uint8_t RawData[13]; 

  double  AccelX_;
  double  AccelY_;
  double  AccelZ_;
  double  Temperature_;
  double  StaticPitch_;
  double  StaticRoll_;  
  
  double CompassX_;
  double CompassY_;
  double CompassZ_;
  uint16_t RHall_;
  
  double SensitivityXY_;
  double SensitivityZ_;
  double TempSlope_;
  
  compass_mode compass_mode_;
  accel_mode accel_mode_;
  bandwidth prev_bw_reg_;   // most recently written bandwidth
};

#endif // BMC050_H_

