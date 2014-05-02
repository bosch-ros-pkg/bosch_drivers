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

/*
 * This class contains a set of parameters that are specific to the 
 * BMP085 sensor.  The class inherits from a generic parent class of 
 * parameters.  I have listed the inherited parameters as comments
 * below.
 * The sensor parameters are stored as members of the class and 
 * accessible to other classes through methods such as 
 * "getSamplingMode()."
 */

#ifndef BMP085_PARAMETERS_H_
#define BMP085_PARAMETERS_H_

// ROS header for debugging outputs
#include <ros/console.h>

//#include <bosch_drivers_common/bosch_drivers_common.hpp>
#include <bosch_drivers_common/bosch_drivers_parameters.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>


using namespace bosch_drivers_common;

class BMP085_parameters: public bosch_driver_parameters
{
public:
 
  /*!
   * \fn BMP085bosch_driver_parameters()
   * \brief  constructor
   */
  BMP085_parameters();
/*!
 * \fn BMP085bosch_driver_parameters()
 * \brief  destructor
 */
  ~BMP085_parameters();


/*!
 * \fn  bool setSamplingMode( sampling_mode mode)
 * \brief set the sensor's sampling mode
 * \param mode the user-requested sampling mode, one of four choices 
 *     given by the enumerated datatype \a sampling_mode.
 * \return a boolean (always true because of the sampling_mode enum)
 */
  //bool setSamplingMode(sampling_mode mode);   

/*!
 * \fn  bool setProtocol(interface_protocol protocol)
 * \param protocol the user-requested interface protocol.
 * \note  this method must be implemeted since it is inheritted.  The
 *     user does not need to call it, however.
 * \return true if the user requests I2C.
 */
  //bool setProtocol(interface_protocol protocol);

/*!
 * \fn  bool setFrequency( int frequency)
 * \brief set the frequency at which the I2C communication will take 
 *     place on the hardware.
 * \param frequency currently only 100000 and 400000 are supported by
 *     all generic hardware interfaces.
 * \return a boolean indicating success.
 */
  //bool setFrequency(int frequency);
  
/*!
 * \fn  bool setPin( uint8_t pin)
 * \brief an unecessary method, since \a pin is an SPI parameter only,
 *     and the sensor only communicates via the I2C protocol.
 *     However, it must be implemented since it is inheritted from
 *     a generic parameters class.
 * \note  user does not need to call this method. It is a dummy.
 * \return always true
 */
  //bool setPin( uint8_t pin);
/*!
 * \fn  interface_protocol getProtocol()
 * \brief returns the interface protocol that this sensor is using to
 *     transmit data.
 * \return  an interface_protocol enumerated datatype from 
 *      bosch_drivers_common.
 */ 
  //interface_protocol  getProtocol();

/*!
 * \fn int getFrequency()
 * \brief returns the communication frequency that the sensor has been
 *     set to be read from the generic hardware interface.
 * \return an int, representing the frequency.
 */
  //int         getFrequency();

/*!
 * \fn int* getFlags()
 * \brief an unecessary method, since the I2C protocol does not need any
 *     additional information (passed down as flags) to be read by
 *     a generic hardware interface.  However, this method must be
 *     implemented since it is inheritted from a generic parameters 
 *     class.
 * \note  user does not need to call this method. It is a dummy.
 * \return the memory address of the class value: \a flags_
 */
  //int*         getFlags();

/*!
 * \fn int  getPin()
 * \brief an unecessary method, since the inheritted \a pin_ value is
 *     only relevant to SPI and GPIO.  However, this method must be 
 *     implemented since it is inheritted from a generic parameters
 *     class.
 * \note  user does not need to call this method. It is a dummy.
 * \return  \a pin_
 */
  //int         getPin();

/*!
 * \fn  sampling_mode getSamplingMode()
 * \brief returns the sampling_mode specified with the 
 *     \a setSamplingMode(mode) method (or the default, \a STANDARD,
 *     if the user did not specifiy a sampling mode).  This 
 *     value is needed to perform the correct calculation from the 
 *     raw sensor values.
 * \return a sampling_mode enumerated datatype.
 */  
  //sampling_mode getSamplingMode();


  // Hardware_Interface hardware_; //|--> we inherit these members from the parameters class!
  // interface_protocol protocol_; //|
  // int frequency_;        //|
  // int pin_;           //|
  // int flags_;          //|

};

#endif // BMP085_PARAMETERS_H_
