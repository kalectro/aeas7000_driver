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

#ifndef AEAS7000_Driver_H_
#define AEAS7000_Driver_H_

// ROS headers for debugging output
#include <ros/console.h>
#include <bosch_drivers_common.hpp>
#include <bosch_drivers_sensor_driver.hpp>
#include <bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

/**
 * \brief Driver to use the AEAS-7000 absolute encoder on a supported serial device.
 *
 */
class Aeas7000Driver: public sensor_driver
{

public:
  /**
   * \brief Constructor
   * \param hw hardware interface to use
   * \param spi_channel SPI channel to use on the hardware device
   */
  Aeas7000Driver( bosch_hardware_interface* hw, uint8_t spi_channel );
 
  // Destructor:
  ~Aeas7000Driver();

  uint8_t getDeviceAddress( void ); 

  /**
   * \brief Reads the current absolute position using SPI
   * \return Absolute value of encoder
   */
  uint16_t get();
	
  /**
   * \brief Initializes the driver and the connected hardware
   * 
   * \return a boolean indicating success
   */
  bool initialize();
  
private:
  uint8_t spi_channel_;
};

#endif // AEAS7000_Driver_H_

