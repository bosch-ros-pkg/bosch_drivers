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

// ROS headers
#include <ros/ros.h>

#include <arduino_interface/arduino_interface.hpp> 
#include <adc_driver/adc_driver.h>

#define REFERENCE_VOLTAGE 5000 // 5000mV = 5V

int main( int argc, char **argv )
{
  
  //ROS initialization and publisher/subscriber setup
  ros::init( argc, argv, "ADC_Driver" );
  ros::NodeHandle nh("~");

  std::string hw_id;
  
  // Get parameters from .launch file or parameter server, or take defaults
  nh.param<std::string>( "hardware_id", hw_id, "/dev/ttyACM0" );

  ArduinoInterface Arduino( hw_id );
  Arduino.initialize();
  
  AdcDriver* adc[3];
  uint32_t voltage[3];
  
  for( int i = 0; i < 3; ++i)
  {
    adc[i] = new AdcDriver( &Arduino, i );
    if( adc[i]->initialize() == false )
    {
      ROS_ERROR("Error initializing ADC driver");
      return -1;
    }
  }
	
  if( !adc[0]->setReference(REFERENCE_VOLTAGE) )
    return -1;

  ros::Rate loop_rate_Hz(1);

  while( nh.ok() )
  {
    for( int i = 0; i < 3; ++i)
    {
      voltage[i] = adc[i]->read();
    }
    ROS_INFO("Read ADC values: channel 0: %u.%uV  channel 1: %u.%uV  channel 2: %u.%uV ", voltage[0] / 1000000, voltage[0] / 1000, voltage[1] / 1000000, voltage[1] / 1000, voltage[2] / 1000000, voltage[2] / 1000 );
          
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }
  
  ROS_WARN( "Closing ADC driver." );
  return 0;  
}
