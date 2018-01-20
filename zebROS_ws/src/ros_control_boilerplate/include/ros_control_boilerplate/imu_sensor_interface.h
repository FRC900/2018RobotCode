
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef HARDWARE_INTERFACE_IMU_SENSOR_INTERFACE_H
#define HARDWARE_INTERFACE_IMU_SENSOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface
{

class ImuSensorHandle
{
public:
  struct Data
  {
    Data()
      : name(),
        frame_id(),
        orientation(0),
        orientation_covariance(0),
        angular_velocity(0),
        angular_velocity_covariance(0),
        linear_acceleration(0),
        linear_acceleration_covariance(0) {}

    std::string name;                       
    std::string frame_id;                   
    double* orientation;                    
    double* orientation_covariance;         
    double* angular_velocity;               
    double* angular_velocity_covariance;    
    double* linear_acceleration;            
    double* linear_acceleration_covariance; 
  };

  ImuSensorHandle(const Data& data = Data())
    : name_(data.name),
      frame_id_(data.frame_id),
      orientation_(data.orientation),
      orientation_covariance_(data.orientation_covariance),
      angular_velocity_(data.angular_velocity),
      angular_velocity_covariance_(data.angular_velocity_covariance),
      linear_acceleration_(data.linear_acceleration),
      linear_acceleration_covariance_(data.linear_acceleration_covariance)
  {}

  std::string getName()                           const {return name_;}
  std::string getFrameId()                        const {return frame_id_;}
  const double* getOrientation()                  const {return orientation_;}
  const double* getOrientationCovariance()        const {return orientation_covariance_;}
  const double* getAngularVelocity()              const {return angular_velocity_;}
  const double* getAngularVelocityCovariance()    const {return angular_velocity_covariance_;}
  const double* getLinearAcceleration()           const {return linear_acceleration_;}
  const double* getLinearAccelerationCovariance() const {return linear_acceleration_covariance_;}

private:
  std::string name_;
  std::string frame_id_;

  double* orientation_;
  double* orientation_covariance_;
  double* angular_velocity_;
  double* angular_velocity_covariance_;
  double* linear_acceleration_;
  double* linear_acceleration_covariance_;
};

class ImuSensorInterface : public HardwareResourceManager<ImuSensorHandle> {};

}

#endif // HARDWARE_INTERFACE_IMU_SENSOR_INTERFACE_H
