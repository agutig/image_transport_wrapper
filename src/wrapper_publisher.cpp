/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, José Miguel Guerrero Hernández.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include "image_transport_wrapper/WrapperPublisher.hpp"

int main(int argc, char * argv[])
  /*
  Esta funcion llama al objeto wrapper en src/image_transport_wrapper/WrapperPublisher.cpp.

  Se encarga de conectarse a un topic de imagen raw y publicarlo en otro (sin modificaciones?)
  

  Se puede llamar como:
  ros2 run image_transport_wrapper wrapper_publisher --ros-args -p topic_in:=/camera/image_raw -p topic_out:=/out/image


  
  
  */
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<wrapper_publisher::WrapperPublisher>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
