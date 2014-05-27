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

//\Author Kai Franke and Philip Roan, Robert Bosch LLC

// ROS headers
#include <ros/ros.h>

#include <arduino_interface/arduino_interface.hpp> 
#include <encoder_driver/encoder_driver.h>

int main( int argc, char **argv )
{
  
  //ROS initialization and publisher/subscriber setup
  ros::init( argc, argv, "Encoder_Driver" );
  ros::NodeHandle nh("~");

  std::string hw_id;
  
  // Get parameters from .launch file or parameter server, or take defaults
  nh.param<std::string>( "hardware_id", hw_id, "/dev/ttyACM0" );

  ROS_INFO("Initializing Arduino");
  ArduinoInterface Arduino( hw_id );
  Arduino.initialize();

  ROS_INFO("Creating fast Encoder");  
  // create new encoder driver using pins 3 and 4
  EncoderDriver* encoder_driver = new EncoderDriver( &Arduino , 4, 3 ); 
  encoder_driver->invertOutput();
  
  ROS_INFO("Initializing Encoder");
  if( encoder_driver->initialize() == false )
  {
    ROS_ERROR("Error initializing encoder driver");
    return -1;
  }

  ROS_INFO("Creating slow Encoder");  
  // create new encoder driver using pins 5 and 6
  EncoderDriver* encoder_driver2 = new EncoderDriver( &Arduino , 5, 6 ); 

  ROS_INFO("Initializing slow Encoder");
  if( encoder_driver2->initialize() == false )
  {
    ROS_ERROR("Error initializing encoder driver");
    return -1;
  }
	
  ros::Rate loop_rate_Hz(1);

  int64_t position, position2;
  ROS_INFO( "Setting initial position" );
  encoder_driver->setPosition( 0 );
  encoder_driver2->setPosition( 0 );

  ROS_INFO("Start reading current position every second...");

  while( nh.ok() )
  {
    position = encoder_driver->getPosition();
    
    position2 = encoder_driver2->getPosition();

    ROS_INFO("Position1: %li    Position2: %li", position, position2);
    
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }
  
  ROS_WARN( "Closing encoder driver." );
  return 0;  
}
