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

//\Author Joshua Vasquez and Philip Roan, Robert Bosch LLC

#ifndef BOSCH_DRIVERS_HARDWARE_INTERFACE_H_
#define BOSCH_DRIVERS_HARDWARE_INTERFACE_H_

#include <string>

#include "bosch_drivers_common.hpp"

namespace bosch_drivers_common
{
  /**
   * \brief Abstract base class for interfaces between sensors and computers.
   *
   * This abstract class provides the generic format so that devices
   * and drivers can communicate over different physical hardware
   * adapters. For example, if gumstix, Sub20, and Arduino all derive from
   * this class, then a single sensor driver can be written independently of all.
   * Concrete implementations of this class provide the actual commands to read
   * and write from the specific hardware interface.
   * Since most interfaces are 8-bit, we use the 8-bit byte as the fundamental
   * unit or size. Commands ask for data in number of bytes.
   */
  class bosch_hardware_interface    
  {
    
  public:                                               
    
    // constructor
    bosch_hardware_interface() {};
    // destructor
    virtual ~bosch_hardware_interface() {};
    
    /**
     * \brief Initialize the hardware interface
     *
     * Any initialization steps that need to be performed by the hardware
     * interface and cannot be done in the constructor should be done here.
     * Some things to configure include the clock/data polarity for SPI and
     * the byte order: little endian or big endian.
     */
    virtual bool initialize () = 0;
    
    /**
     * \brief Read a given number of bytes
     *
     * Communication with a device requires reading from locations in that device's
     * memory. This function retrieves a number of bytes starting at a known address
     * in a device. If the particular device does not store measurements in memory
     * then it needs to be properly configured in the initialize() function.
     * \param  device_address  The starting register address in the device's memory.
     * \param  protocol
     * \param  frequency       The protocol frequency
     * \param  flags
     * \param  reg_address     The register address
     * \param  data            A pointer to the memory location in the host where the data
     *                         will be stored. The user must make sure space is available.
     *                         This data should be an array.
     * \param  num_bytes       The requested number of bytes to read from the device.
     * \return                 The number of bytes actually read from the device or an
     *                         error code.
     *
     * \note  For SPI transactions, the \p device_address is the chip select. If
     * the device address is \p NULL_DEVICE, then the hardware interface should
     * read from the bus without changing any chip select lines.
     * \note  For GPIO transactions, the \p device_address is the pin or port.
     */
    virtual ssize_t read( int device_address, 
                          interface_protocol protocol, 
                          int frequency, 
                          int* flags, // relevant for SPI communiction
                          uint8_t reg_address, 
                          uint8_t* data, 
                          size_t num_bytes ) = 0;               
        
    /**
     * \brief Write the provided data, as bytes
     * 
     * Communication with a device requires writing to locations in that device's
     * memory. This function sends a number of bytes starting at a known address
     * in a device. If the particular device does not store commands in memory
     * then it needs to be properly configured in the initialize() function.
     * \param  device_address  The starting register address in the device's memory.
     * \param  protocol
     * \param  frequency       The protocol frequency
     * \param  flags
     * \param  reg_address     The register address
     * \param  data            A pointer to the memory location in the host where the data
     *                         will be stored. The user must make sure space is available.
     *                         This data should be an array.
     * \param  num_bytes       The requested number of bytes to read from the device.
     * \return                 The number of bytes actually read from the device or an
     *                         error code.
     *
     * \note  If \p num_bytes is zero, then the hardware should just write the
     * register as a single byte. In this case, the pointer data is irrelevant.
     * This can be used to write single bytes.
     * \note  For SPI transactions, the \p device_address is the chip select. If
     * the device address is \p NULL_DEVICE, then the hardware interface should
     * read from the bus without changing any chip select lines.
     * \note  For GPIO transactions, the \p device_address is the pin or port.
     */
    virtual ssize_t write( int device_address, 
                           interface_protocol protocol, 
                           int frequency,
                           int* flags, 
                           uint8_t reg_address, 
                           uint8_t* data, 
                           size_t num_bytes ) = 0;      

    /**
     * \brief Check if a communication protocol is supported by this hardware interface.
     */
    virtual bool supportedProtocol( interface_protocol protocol ) = 0;
  
    /**
     * \brief Return an identifier for this hardware interface.
     *
     * This identifier should be able to uniquely identify this interface when
     * multiple copies of this interface exist.
     */
    virtual std::string getID() = 0;                  
  };
}

#endif //BOSCH_DRIVERS_HARDWARE_INTERFACE_H_

































