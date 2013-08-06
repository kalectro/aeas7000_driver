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

//\Author Kai Franke

#include "aeas7000_driver/aeas7000_driver.h"

Aeas7000Driver::Aeas7000Driver( bosch_hardware_interface* hw, uint8_t spi_channel ): sensor_driver( hw )
{
  spi_channel_ = spi_channel;  
}

Aeas7000Driver::~Aeas7000Driver()
{
}

uint8_t Aeas7000Driver::getDeviceAddress()
{
  // the answer to all questions and one less
  return 41;
}

bool Aeas7000Driver::initialize()
{  
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("Aeas7000Driver::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  return true;
}

uint16_t gray_to_binary(uint16_t gray)
{
    uint16_t result = gray & (1 << 15);
    
    for( int i = 14; i >= 0; --i )
    {
      result |= (gray ^ (result >> 1)) & (1 << i);
    }
    return result;
}

uint16_t Aeas7000Driver::get()
{
  uint8_t spi_slave_select = spi_channel_;
  uint8_t bit_order = 0;
  uint8_t mode = 0;

  int flags[1] = { (spi_slave_select << 4) | (bit_order << 2) | mode };
  int frequency = 2e6;
  uint8_t num_bytes = 2;
  uint8_t data[num_bytes];
  
  if( hardware_->read( this->getDeviceAddress(), SPI, frequency, flags, 0, data, num_bytes ) < 0 )
  {
    ROS_ERROR("Aeas7000Driver::get(): could not read input");
    return false;
  } 
  // compose read position as Gray Code
  uint16_t position = data[0];
  position = (position << 8) | data[1];
  
  // return position as binary
  return gray_to_binary(position);
}


