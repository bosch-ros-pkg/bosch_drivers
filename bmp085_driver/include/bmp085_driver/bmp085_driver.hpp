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


#ifndef BMP085_DRIVER_H_
#define BMP085_DRIVER_H_

#include <cmath>
#include <unistd.h> // for the sleep function

#include <bosch_drivers_common/bosch_drivers_sensor_driver.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>

//#include "bmp085_parameters.hpp"

using namespace bosch_drivers_common;

/**
 * \brief      a BMP085 driver that implements the devices main 
 * capabilities.
 *
 * \details     This class provides access to both the temperature
 * and pressure data.  
 * \warning   This class implements it's methods through a generic
 * hardware interface.
 */
class BMP085: public sensor_driver
{
public:

/**
 * \brief Pressure Address register values.
 *
 * Only \a PRESSURE_OSRS_0 is used in the class, since the rest can be deduced
 *  by adding(oss << 6) as offset from PRESSURE_OSRS_0.
 */
  static const uint8_t PRESSURE_OSRS_0 = 0x34; // 4.5 [ms]
  static const uint8_t PRESSURE_OSRS_1 = 0x74; // 7.5 [ms]
  static const uint8_t PRESSURE_OSRS_2 = 0xB4; // 13.5 [ms]
  static const uint8_t PRESSURE_OSRS_3 = 0xF4; // 25.5 [ms]
 
  static const uint8_t INPUT_REG_ADDRESS = 0xF4;
  
  // UT is the temperature data. To calculate it, 
  //   UT = MSB<<8 + LSB
  // UP is the pressure data.  To calculate it, 
  //   UP = (MSB<<16 + LSB<<8 +XLSB) >> (8 - oss)
  // where oss is the oversample setting (0,1,2, or 3). 
  static const uint8_t MEAS_OUTPUT_MSB  = 0xF6;
  static const uint8_t MEAS_OUTPUT_LSB  = 0xF7;
  static const uint8_t MEAS_OUTPUT_XLSB = 0xF8;

  /*!
   * \enum sampling_mode
   * \brief configurable sampling mode. Higher resolution settings will
   *     result in a longer conversion time on the sensor before the 
   *     value can be read.
   */
  enum sampling_mode
  {
    /*! 
     * \var ULTRA_LOW_POWER
     * \note conversion_time: 4.5 [ms] */ 
    ULTRA_LOW_POWER       = 0,
    /*! 
     * \var STANDARD
     * \note conversion_time: 7.5 [ms] */ 
    STANDARD              = 1,
    /*! 
     * \var HIGH
     * \note conversion_time: 13.5 [ms] */ 
    HIGH                  = 2,
    /*! 
     * \var ULTRA_HIGH_RESOLUTION
     * \note conversion_time: 25.5 [ms] */ 
    ULTRA_HIGH_RESOLUTION = 3
  };


  BMP085( bosch_hardware_interface* hw );
  ~BMP085(); 

  // inheritted from sensor_driver
  //uint8_t getDeviceAddress(); // depends on protocol_.
  
  /**
   * \brief sends a prompt to initialize the connected hardware device so 
   *     that communication can begin with the sensor.
   *
   * This method also sets class constants based on the user-defined (or 
   *     default) parameters set beforehand.
   * \return a boolean indicating successful hardware initialization.
   */
  bool initialize();
 
  /** 
   * \brief First acquires the raw temperature; then acquires the raw
   *      pressure.
   *
   * Computes the calculated temperature and
   *      pressure based on the two raw sensor readings.
   * The pressure calculation is dependent on the temperature.
   * \return  A boolean indicating success
   */
  bool takeMeasurement();
  
  /**
   * \brief acquires the sensor's raw pressure data depending on the 
   *     sampling mode and computes the calculated pressure using the 
   *     most recent temperature measurement.
   *
   * \warning  it is critical that the temperature is measured 
   *       immediately beforehand for an accurate pressure value.
   * \return a boolean indicating success.
   */
  bool getPressureData( void );
  
  /**
   * \brief acquires the sensors raw temperature data and calculates the
   *     calculated temperature data.
   *
   * \return a boolean indicating success.
   */
  bool getTemperatureData( void );
  
  /**
   * \return the most recent calculated temperature data.
   */ 
  double getTemperature();

  /**
   * \return the most recent calculated pressure data.
   */
  double getPressure();

 /**
  * \brief Compute the altitude using the latest pressure data and the 
  *     reference pressure, which is \a pressure_at_sea_level_.
  *
  * \warning reference pressure must be changed beforehand if it differs 
  *      from the default value: 101.325 [kPa].
  * \return the calculated altitude in [m].
  */ 
  double getAltitude( void );
 
  /**
   * \brief calculate the altitude given an input pressure and using the
   *     class's reference pressure, \a pressure_at_sea_level_.
   *
   * \param input_pressure the current pressure.
   * \return the pressure in [kPa].
   */
  double calcAltitude( double input_pressure );

  /**
   * \brief set the reference pressure to another value different from the
   *     default value: 101.325 [kPa]
   *
   * \param input_pressure the new reference pressure in [kPa]
   */
  void setPressureAtSeaLevel( double pressure );

  bool setSamplingMode( sampling_mode mode );
  sampling_mode getSamplingMode();

  uint8_t getDeviceAddress();
  bool setProtocol( interface_protocol protocol );
  interface_protocol getProtocol();

  bool setFrequency( int frequency );
  int getFrequency();

private:
  // BMP085 Register Addresses
  static const uint8_t DEVICE_ADDRESS = 0x77; // the address without the READ/WRITE bit.
 
  static const uint8_t ADDRESS_WRITE = 0xEE;
  static const uint8_t ADDRESS_READ  = 0xEF;
 
  static const uint8_t ADDRESS_AC1_MSB = 0xAA;
  static const uint8_t ADDRESS_AC1_LSB = 0xAB;
  static const uint8_t ADDRESS_AC2_MSB = 0xAC;
  static const uint8_t ADDRESS_AC2_LSB = 0xAD;
  static const uint8_t ADDRESS_AC3_MSB = 0xAE;
  static const uint8_t ADDRESS_AC3_LSB = 0xAF;
  static const uint8_t ADDRESS_AC4_MSB = 0xB0;
  static const uint8_t ADDRESS_AC4_LSB = 0xB1; 
  static const uint8_t ADDRESS_AC5_MSB = 0xB2;
  static const uint8_t ADDRESS_AC5_LSB = 0xB3;
  static const uint8_t ADDRESS_AC6_MSB = 0xB4;
  static const uint8_t ADDRESS_AC6_LSB = 0xB5;
  static const uint8_t ADDRESS_B1_MSB  = 0xB6;
  static const uint8_t ADDRESS_B1_LSB  = 0xB7;
  static const uint8_t ADDRESS_B2_MSB  = 0xB8;
  static const uint8_t ADDRESS_B2_LSB  = 0xB9;
  static const uint8_t ADDRESS_MB_MSB  = 0xBA;
  static const uint8_t ADDRESS_MB_LSB  = 0xBB;
  static const uint8_t ADDRESS_MC_MSB  = 0xBC;
  static const uint8_t ADDRESS_MC_LSB  = 0xBD;
  static const uint8_t ADDRESS_MD_MSB  = 0xBE;
  static const uint8_t ADDRESS_MF_LSB  = 0xBF;
 
  /**
   * \brief Write this value into INPUT_REG_ADDRESS to prompt the sensor
   *      for temperature.
   */
  static const uint8_t ADDRESS_TEMP_REQUEST  = 0x2E;

  /**
   * \note  this value is set to the value requested in the parameters 
   *     upon calling the \a initialize() method.
   * \note  if no value is called in the parameters, the oss mode is set 
   *     to the default value: \a STANDARD.
   */
  /*!
   * \var oss_
   * \brief a class member used to store the user-defined sampling mode
   *     so that it can be passed to the driver with the
   *     \a getSamplingMode() method.
   */
  sampling_mode oss_;


  /**
   * \brief The following values are calibration constants specific to 
   *     each individual sensor.
   *
   * They are stored in the sensors EEPROM
   *     and are read out and stored to these class members upon 
   *     calling the \a initialize() method.
   * \note  these values are necessary to calculate the pressure.
   * \note  B5 is determined by the latest temperature measurement, so its
   *     value changes each time the temperature calculation changes.
   */
  short AC1, AC2, AC3;
  unsigned short AC4, AC5, AC6;
  short B1, B2;
  long B5; 
  short MB, MC, MD;
  long UT, UP;
  
  /**
   * The raw temperature in [C].
   */
  long T;
  
  /** 
   * The raw pressure in [kPa]
   */
  long p; 

  /**
   * \brief the class's reference pressure used to perform the altitude 
   *     calculation.
   */
  double pressure_at_sea_level_;

  /**
   * \brief the calculated altitude in [m]
   */
  double altitude_;

  /**
   * \brief the latest calculated temperature measurement determined from
   *     the raw temperature measurement from the sensor.
   * \note  units: [C]
   */
  double temperature_; 

  /**
   * \brief the latest calculated pressure measurement determined from
   *     both the raw temperature and raw pressure measurements.
   * \note units: [kPa]
   */
  double pressure_;

  bool writeToReg( uint8_t reg, uint8_t value );
};

#endif // BMP085_DRIVER_H_
