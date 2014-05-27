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
#include <gpio_driver/gpio_driver.h>


#define OUTPUT_PIN 4
#define INPUT_PIN 3

int main( int argc, char **argv )
{
  
  //ROS initialization and publisher/subscriber setup
  ros::init( argc, argv, "GPIO_Driver" );
  ros::NodeHandle nh("~");

  std::string hw_id;
  
  // Get parameters from .launch file or parameter server, or take defaults
  nh.param<std::string>( "hardware_id", hw_id, "/dev/ttyACM0" );

  ArduinoInterface Arduino( hw_id );

  GpioDriver* gpio_input = new GpioDriver( &Arduino, INPUT_PIN );
  GpioDriver* gpio_output = new GpioDriver( &Arduino, OUTPUT_PIN );
	
  if( gpio_input->initialize() == false || gpio_output->initialize() == false )
  {
    ROS_ERROR("Error initializing GPIO driver");
    return -1;
  }
  
  ros::Rate loop_rate_Hz(10);
  loop_rate_Hz.sleep();
  
  bool value = 1;
  bool value2 = 0;
	
  while( nh.ok() )
  {
    if( !gpio_output->setOutput( value ) )
    {
      return -1;
    }
      
    value = !value;
    
    value2 = gpio_input->getInput( bosch_drivers_common::FLOATING );
    ROS_INFO("Read value %i", value2);
      
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }
  
  ROS_WARN( "Closing GPIO driver." );
  return 0;  
}
