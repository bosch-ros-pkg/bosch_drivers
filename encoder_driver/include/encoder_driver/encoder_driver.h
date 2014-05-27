/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Robert Bosch LLC.
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

//\Author Kai Franke, Robert Bosch LLC

#ifndef ENCODER_DRIVER_H_
#define ENCODER_DRIVER_H_

// ROS headers for debugging output
#include <ros/console.h>

#include <bosch_drivers_common/bosch_drivers_common.hpp>
#include <bosch_drivers_common/bosch_drivers_sensor_driver_internal.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

/**
 * \brief Driver to use a motor encoder on a supported serial device.
 *
 * This class enables the user to create an instance of an encoder on the
 * connected serial device itself. The device will keep track of the encoder value
 * using external interrupts if available on the selected pin. To contruct the
 * encoder object the user needs to define two pins to be used by the device.
 * These pins will automatically enable the internal pullup resistor and expect
 * the connected encoder to pull the input to ground or drive in both directions.
 * It is highly recommended to use a pin with external interrupt support at least
 * for encoder1_pin. If not, the encoder object will need to be polled frequently
 * to detect any changes.
 * See http://www.pjrc.com/teensy/td_libs_Encoder.html for more information
 */
class EncoderDriver: public sensor_driver_internal
{
  
public:
  /**
   * \brief Constructor: also constructs an encoder object on the serial device itself
   * \note the maximum number of encoder objects is limited to 16 even after destruction of objects
   * \param hw defines where the serial device is mounted like /dev/ttyACM0
   * \param encoder1_pin first encoder pin on the serial device. This should be a pin supporting external interrupts
   * \param encoder2_pin second encoder pin on the serial device.
   */
  EncoderDriver( bosch_hardware_interface* hw, uint8_t encoder1_pin, uint8_t encoder2_pin );
  
  /**
   * \brief Constructor: does not construct a new object on the serial device but uses the one defined by \a encoder_id
   * \param hw defines where the serial device is mounted like /dev/ttyACM0
   */
  EncoderDriver( bosch_hardware_interface* hw, uint8_t encoder_id );
 
  // Destructor:
  ~EncoderDriver();

  // Public Driver Methods:
  uint8_t getDeviceAddress( void );
  bool setDeviceAddress( uint8_t address);
 
  bosch_driver_parameters getParameters();
  bool setParameters( bosch_driver_parameters parameters );

  /**
   * \brief Sets the current motor encoder position to \a position
   * \param position the position to set the encoder count to
   * \return true if encoder write was successful or false if not
   */
  bool setPosition( int32_t position );
  
  /**
   * \brief Gets the current motor encoder position from the connected hardware device
   * \note Make sure to call getPosition at least every 1 billion encoder ticks for the connected hardware works with a int32_t. An overflow is handles by this function
   * \return latest encoder position in encoder ticks
   */
  int64_t getPosition();

  bool zero();

  /**
   * \brief Initializes the driver and the connected hardware
   * 
   * \return a boolean indicating success
   */
  bool initialize();
  
  /**
   * \brief Getter method to get the encoder id of the object on the connected serial device
   * 
   * \return encoder id on the serial device [0..15]
   */
  uint8_t getEncoderID();
  
  /**
   * \brief Inverts the encoder output
   * 
   * \todo this should not be neccessary and should be possible by changing the encoder pins, but it does not work :(
   */
  void invertOutput();
  
private:

  // these two should be parameters
  uint8_t _encoder1_pin;
  uint8_t _encoder2_pin;

  int64_t _overflow;
  int32_t _last_position;
  int invert_;

  static const uint8_t MAX_ENCODERS = 16;
  static bool encoders_[MAX_ENCODERS];

  // Returns true if the ID is valid. id contains the next available id. The id is reserved at this step, and must be released if the encoder object cannot be created on the hardware interface or when the encoder opbject is deleted. 
  bool getNextID( uint8_t* id );
};

#endif // ENCODER_DRIVER_H_
