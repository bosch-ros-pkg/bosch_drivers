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

//\Author Joshua Vasquez, Philip Roan, Bosch LLC

#ifndef BMG160_H_
#define BMG160_H_

#include <iostream> // for the calibration routine
#include <cmath>
#include <unistd.h> // for the sleep function

// ROS header for output
#include <ros/console.h>

#include <bosch_drivers_sensor_driver.hpp>
#include <bosch_drivers_hardware_interface.hpp>
#include "bmg160_parameters.hpp"

#include <bmg160_driver/bmg160meas.h>

using namespace bosch_drivers_common;

/**
 * \brief           a bmg160 driver that implements the device's main 
 * capabilities.
 *
 * \details         This class provides access to the acceleration 
 * temperature, and Magnetic field data.  Routines have been written to 
 * change settings, such as bandwidth and range, but \a not all 
 * capabilities of the sensor have been implemented. 
 * \note            motion-detection not implemented.
 * \note            fifo buffer not used.
 * \warning         This class implements its methods through a generic
 * hardware interface.
 */

class BMG160: public sensor_driver, public BMG160Parameters
{
 
public:
  
  /// Register bitflags and commands:
  static const uint8_t SOFTRESET_CMD = 0xB6;

 /**
  * \brief ADDRESS_FILTER bitflags
  *
  * The following members are the relevant bitflag offsets of 
  *     the ADDRESS_FILTER register:
  */
  static const uint8_t DATA_HIGH_BW = 7;  // <7>
  
  /**
   * \brief ADDRESS_ENABLE_FAST_CAL bitflags
   *
   * The following members are the relevant bitflag offsets of 
   *     the ADDRESS_ENABLE_FAST_CAL register:
   */  
  static const uint8_t fast_offset_en   = 3;
  static const uint8_t fast_offset_en_z = 2;
  static const uint8_t fast_offset_en_y = 1;
  static const uint8_t fast_offset_en_x = 0;
  
  /**
   * \brief ADDRESS_FILTERED_CAL_FAST bitflags
   *
   * The following members are the relevant bitflag offsets of 
   *     the ADDRESS_FILTERED_CAL_FAST register:
   */
  static const uint8_t fast_offset_unfilt  = 7;

  /**
   * \brief ADDRESS_ENABLE_CAL bitflags
   *
   * The following members are the relevant bitflag offsets of 
   *     the ADDRESS_ENABLE_CAL register:
   */
  static const uint8_t slow_offset_th   = 6; // <7:6>
  static const uint8_t slow_offset_dur  = 3; // <5:3>
  static const uint8_t slow_offset_en_x = 0;
  static const uint8_t slow_offset_en_y = 1;
  static const uint8_t slow_offset_en_z = 2;

  /**
   * \brief ADDRESS_FILTERED_CAL bitflags
   *
   * The following members are the relevant bitflag offsets of 
   *     the ADDRESS_FILTERED_CAL register:
   */
  static const uint8_t slow_offset_unfilt = 5;
  
  /**
   * \brief ADDRESS_NVM bitflags
   *
   * The following members are the relevant bitflag offsets of 
   *     the ADDRESS_NVM register:
   */
  static const uint8_t nvm_load = 3;

  BMG160( bosch_hardware_interface* hw );
  ~BMG160();

  /**
   * \brief   An invalid request.  It is necessary, since this driver
   *          inherits from the sensor_driver class.
   *
   * See the getAccelAddress() and getCompassAddress() methods in the 
   *          corresponding parameters class. 
   * \return 0  An invalid address, since each sensor has its own address. 
   */
  uint8_t getDeviceAddress(); 
  
  /**
   * \brief Initializes the sensor with all user-defined
   *        preferences indicated in the parameters.
   *
   * \return a boolean indicating success
   */
  bool initialize();

  /**
   * \brief Retrieves all available raw gyro data and stores 
   *          the calculated value to class variables: \a GyroX_, 
   *          \a GyroY_, \a GyroZ_, \a Temperature_.
   *
   * \return  a boolean indicating success.
   */
  bool takeMeasurement();

  /**
   * \brief   resets the accelerometer, putting it back into normal mode
   *          and clearing its image.
   *
   * \return  a boolean indicating whether or not the hardware sent the 
   *          reset command.
   */
  bool softReset();

  /**
   * \brief   a method that changes the accelerometer's range by writing 
   *          to the appropriate sensor register.  The range is specified 
   *          by the \a  setRange(range) in the inheritted parameter's
   *          class.
   *
   * The range will be changed to the default range if the user
   *          has not specified a range.
   * \return  a boolean indicating success.
   */
  bool changeRange();                  

  /**
   * \brief writes to the sensor's register in memory to choose whether or 
   *        not to filter the gyro data.
   *
   * The data will be set as unfiltered if the user has not 
   * specified an alternate request.
   * See the datasheet (page 40) for more details on register 0x13.
   * \param request   a boolean indicating the user's choice of filtered
   *        (\a true ) or unfiltered ( \a false ).
   * \return a boolean indicating whether or not the sensor register has 
   *         adjusted to reflect the request.
   */
  bool filterData( bool request );         
  
 /**
  * \brief changes the gyro's bandwidth by writing to the 
  *        appropriate sensor register.
  *
  * The bandwidth is specified by
  *        the setBandwidth(bandwidth) method in the inheritted 
  *        parameters class.
  * The bandwidth will be changed to the default bandwidth if the 
  *        user has not specified a bandwidth with
  *        \a setBandwidth(bandwidth) in the parameters class.
  * See the datasheet for more details on register 0x10 
  * \return a boolean indicating whether or not the sensor register has
  *         adjusted to reflect the requested bandwidth.
  */
  bool changeBandwidth();  

  /**
   * \brief   retrieves the raw gyro x-axis data and stores the
   *          calculated value to the class variable: \a GyroX_
   *
   * If the application requires more than just one axis of data,
   *          use the \a takeMeasurement() instead, as it is much more
   *          efficient.
   * \return  \a GyroX_
   */
  double    getGyroX();

 /**
  * \brief   retrieves the raw gyro y-axis data and stores the
  *          calculated value to the class variable: \a GyroY_
  *
  * If the application requires more than just one axis of data,
  *          use the \a takeMeasurement() instead, as it is much more
  *          efficient.
  * \return  \a GyroY_
  */
  double getGyroY();

  /**
   * \brief   retrieves the raw gyro z-axis data and stores the
   *          calculated value to the class variable: \a GyroZ_
   *
   * If the application requires more than just one axis of data,
   *          use the \a takeMeasurement() instead, as it is much more
   *          efficient.
   * \return  \a GyroZ_
   */
  double getGyroZ();

  /**
   * \brief   retreives the raw gyro temperature and stores the calculated
   *          value in the class variable \a Temperature_.
   *
   * If the application requires more than just the temperature,
   *          use the \a takeMeasurement() method instead, as it is much 
   *          more efficient.
   * \return \a Temperature_
   */
  double getTemperature();

  /**
   * \brief   calibrates the sensor assuming it is in a static position.
   *
   * Calibration parameters are defaults, which are as follows:
   * A calibration should be necessary only if the 
   *          gyro does not produce accurate measurements.  The 
   *          sensor should already come from the factory with
   *          well-defined pre-calibrated constants.
   * \note This calibration does NOT permanently override the sensor's
   *          pre-calibrated constants. 
   */
  void SimpleCalibration();

  /**
   * \brief   reads \a num_bytes starting at register \a reg and stores
   *          the data into the array \a data_array.
   *
   * \return  a boolean indicating success.
   */
  bool readSensorData( uint8_t reg, uint8_t* data, uint8_t num_bytes );

  /**
   * \param   uint8_t* raw_offset_data the name of the input array 
   *          starting at \a ADDRESS_OFFSETS.
   * \param   uint8_t*  clean_offset_data the name of the output array
   *          of 3 unsigned 16-bit ints containing the 12-bit offset 
   *          values.
   */ 
  void computeOffsets( uint8_t* raw_offset_data, int16_t* clean_offset_data );

  /**
   * \brief   read the offset registers, format the 12-bit values 
   *          correctly, and print the values for all three axes.
   *
   * \return  a boolean indicating success
   */
  bool printOffsets();


  /// BMG160 REGISTER ADDRESSES 
  
  /** \var static const uint8_t SLAVE_ADDRESS0  
   *  \brief  the i2c device address with the chip's SDO pin connected
   *          to GND .            */
  static const uint8_t SLAVE_ADDRESS0           = 0x68;
  /** \var static const uint8_t SLAVE_ADDRESS1  
   *  \brief  the i2c device address with the chip's SDO pin connected 
   *          to VDD.           */
  static const uint8_t SLAVE_ADDRESS1            = 0x69;
  
  static const uint8_t ADDRESS_GYRO_X_LSB        = 0x02;
  static const uint8_t ADDRESS_GYRO_Y_LSB        = 0x04;
  static const uint8_t ADDRESS_GYRO_Z_LSB        = 0x06;
  static const uint8_t ADDRESS_TEMPERATURE       = 0x08;
  
  static const uint8_t ADDRESS_RANGE             = 0x0F;
  static const uint8_t ADDRESS_BANDWIDTH         = 0x10;
  static const uint8_t ADDRESS_FILTER            = 0x13;
  static const uint8_t ADDRESS_SOFTRESET         = 0x14;
  static const uint8_t ADDRESS_FILTERED_CAL      = 0x1A;
  static const uint8_t ADDRESS_FILTERED_CAL_FAST = 0x1B;
  static const uint8_t ADDRESS_ENABLE_CAL        = 0x31;
  static const uint8_t ADDRESS_ENABLE_FAST_CAL  = 0x32;
  static const uint8_t ADDRESS_NVM            = 0x33;

  /** \var static const uint8_t ADDRESS_OFFSETS
   *  \brief  the starting address of the offset values that are adjusted 
   *          in calibration.
   *
   * \note    4 bytes of data starting from this address.
   * \note    12-bit offset values:
   * \warning Data is not sequential.  See the following notes: 
   * \note  X: <11:4> in 0x37<7:0>, <3:2> in 0x36<7:6>, <1:0> in 0x3A<3:2> 
   * \note  Y: <11:4> in 0x38<7:0>, <3:1> in 0x36<5:3>, <0> in 0x3A<1>
   * \note  Z: <11:4> in 0x39<7:0>, <3:1> in 0x36<2:0>, <0> in 0x3A<0>
   */
  static const uint8_t ADDRESS_OFFSETS          = 0x36; 

  double GyroX_;
  double GyroY_;
  double GyroZ_; 
  double Temperature_;  
  double TempSlope_;
  
private:
  bool readReg( uint8_t reg, uint8_t* value );
  bool writeToReg( uint8_t reg, uint8_t value );
  bool writeToRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected );
};

#endif // BMC050_H_
