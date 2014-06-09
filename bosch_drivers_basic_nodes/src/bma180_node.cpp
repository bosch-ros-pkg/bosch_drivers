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

#include <cmath> // for M_PI

// ROS headers
#include <ros/ros.h>
#include <ros/time.h>

#include <bma180_driver/bma180.hpp>
#include <sub20_interface/sub20_interface.hpp> 

// ROS message
#include <bma180_driver/bma180_measurement.h>

/**
 * This class is an example node that publishes the data from several
 * BMA180s.  In this case, the BMA180s are all connected on the same
 * hardware interface, a sub20.  However, multiple hardware interfaces
 * can be instantiated.  In this way, the user can create a node that 
 * publishes all BMA180 data regardless of the hardware interface that
 * it is connected to.
 **/
int main( int argc, char **argv )
{
  //ROS initialization and publisher/subscriber setup
  ros::init(argc, argv, "BMA180_node");
  ros::NodeHandle nh;

  // User configurable parameters

  /**
   * \brief The unique identifier for the hardware interface connecting the
   * BMA180 sensors to the computer. 
   */
  std::string hw_id;
  /**
   *
   * How many BMA180s are connected to the interface. In i2c mode, maximum
   * is 2 on one device (using separate addresses). in SPI mode, maximum
   * is # of chipselect lines on one device.
   */
  int number_of_bma180_sensors;
  int publish_rate_Hz;

  // Get parameters from .launch file or parameter server, or take defaults
  nh.param<int>( "/bma180_node/number_of_bma180_sensors", number_of_bma180_sensors, 1 );
  nh.param<int>( "/bma180_node/publish_rate_Hz", publish_rate_Hz, 100 );
  nh.param<std::string>( "/bma180_node/hardware_id", hw_id, "0208" );


  Sub20Interface sub20( hw_id );   
  std::vector<BMA180*> bma180_chain(number_of_bma180_sensors);

  // Initialize the BMA180s
  for( int i = 0; i < number_of_bma180_sensors; i++ )
  {
    // instantiate a sensor and pass in a pointer to its hardware interface
    BMA180* Accelerometer = new BMA180( &sub20 ); 

    // Adjust sensor settings:
    Accelerometer->setAccelerationRange( BMA180::RANGE_8 );
    Accelerometer->setBandwidth( BMA180::BW_150 );

    // SPI configuration
    Accelerometer->setProtocol( SPI );
    Accelerometer->setFrequency( 125000 );
    Accelerometer->setDeviceAddress( i );

    // I2C configuration
    //Accelerometer->setProtocol(I2C); // CSB connected to VDD
    //Accelerometer->setFrequency( 400000 );
    //Accelerometer->setSlaveAddressBit( 0 );

    if( Accelerometer->initialize() == false )
    {
      ROS_ERROR( "BMA180 device: %d initialization failed.", i );
      //return 0;
    }

     // Add the sensor to the sensor chain
    bma180_chain.push_back( Accelerometer );
  }
  
  // Set up ROS message and publisher
  bma180_driver::bma180_measurement msg;
  ros::Publisher bma180_pub = nh.advertise<bma180_driver::bma180_measurement>( "BMA180_data", 20 );

  //set loop rate for measurement polling
  ros::Rate loop_rate_Hz( publish_rate_Hz ); 
    
  double pitch, roll;
  
  // Run sensor node
  while( nh.ok() )
  {
    // Data goes in a standard vector
    for( int j = 0; j < number_of_bma180_sensors; j++ )
    {
      // Read measurements from sensors
      if( bma180_chain[j]->takeMeasurement() == false )
      {
	ROS_ERROR( "BMA180 device %d failed to acquire measurements.", j );
	//return 0;
      }

      roll = bma180_chain[j]->getStaticRoll() * (180.0/M_PI);   
      pitch = bma180_chain[j]->getStaticPitch() * (180.0/M_PI);
  
      msg.AccelerationX.push_back( bma180_chain[j]->getAccelX() );
      msg.AccelerationY.push_back( bma180_chain[j]->getAccelY() );
      msg.AccelerationZ.push_back( bma180_chain[j]->getAccelZ() );
   
      msg.Temperature.push_back( bma180_chain[j]->getTemperature() );
   
      msg.staticRoll.push_back( roll );
      msg.staticPitch.push_back( pitch );
    }
  
    // publish message
    msg.header.stamp = ros::Time::now();
    bma180_pub.publish( msg ); 

    // clear vectors so data doesn't accumulate
    msg.AccelerationX.clear();
    msg.AccelerationY.clear();
    msg.AccelerationZ.clear();
    msg.Temperature.clear();
    msg.staticRoll.clear();
    msg.staticPitch.clear();
    
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }

  ROS_WARN( "Closing BMA180 driver." );
  return 0;
}
