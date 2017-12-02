///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
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
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Wim Meeussen
 */

#include <algorithm>
#include <cstddef>

#include "talon_state_controller/talon_state_controller.h"
#include "talon_state_controller/TalonState.h"

namespace talon_state_controller
{

  bool TalonStateController::init(hardware_interface::TalonStateInterface* hw,
                                  ros::NodeHandle&                         root_nh,
                                  ros::NodeHandle&                         controller_nh)
  {
    // get all joint names from the hardware interface
    const std::vector<std::string>& joint_names = hw->getNames();
    num_hw_joints_ = joint_names.size();
    for (unsigned i=0; i<num_hw_joints_; i++)
      ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    // realtime publisher
    realtime_pub_.reset(new
    realtime_tools::RealtimePublisher<talon_state_controller::TalonState>(root_nh, "talon_states",
    4));

    // get joints and allocate message
    for (unsigned i=0; i<num_hw_joints_; i++){
      talon_state_.push_back(hw->getHandle(joint_names[i]));
      realtime_pub_->msg_.position.push_back(0.0);
      realtime_pub_->msg_.speed.push_back(0.0);
      realtime_pub_->msg_.output_voltage.push_back(0.0);
      realtime_pub_->msg_.can_id.push_back(0);
    }
    addExtraJoints(controller_nh, realtime_pub_->msg_);

    return true;
  }

  void TalonStateController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }

  void TalonStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

      // try to publish
      if (realtime_pub_->trylock()){
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        // populate joint state message:
        // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
        // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
        /*
			double getPosition(void)      const {return position_;}
			double getSpeed(void)         const {return speed_;}
			double getOutputVoltage(void) const {return output_voltage_;}
			int    getCANID(void)         const {return can_id_;}
			double getOutputCurrent(void) const {return output_current_;}
			double getBusVoltage(void)    const {return bus_voltage_;}
			double getPidfP(void)	      const {return pidf_p_;}
			double getPidfI(void)	      const {return pidf_i_;}
			double getPidfD(void)	      const {return pidf_d_;}
			double getPidfF(void)	      const {return pidf_f_;}
			int getClosedLoopError(void)  const {return closed_loop_error_;}
			int getFwdLimitSwitch(void)   const {return fwd_limit_switch_closed_;}
			int getRevLimitSwitch(void)   const {return rev_limit_switch_closed_;}
			TalonMode getTalonMode(void)  const {return talon_mode_;}
            */
        realtime_pub_->msg_.header.stamp = time;
        for (unsigned i=0; i<num_hw_joints_; i++){
          realtime_pub_->msg_.position[i] = talon_state_[i]->getPosition();
          realtime_pub_->msg_.speed[i] = talon_state_[i]->getSpeed();
          realtime_pub_->msg_.output_voltage[i] = talon_state_[i]->getOutputVoltage();
          realtime_pub_->msg_.can_id[i] = talon_state_[i]->getCANID();
          realtime_pub_->msg_.output_current[i] = talon_state_[i]->getOutputCurrent();
          realtime_pub_->msg_.bus_voltage[i] = talon_state_[i]->getBusVoltage();
          //publish the array of PIDF values
          realtime_pub_->msg_.pid_p1[i] = talon_state_[i]->getPidfP(0);
          realtime_pub_->msg_.pid_i1[i] = talon_state_[i]->getPidfI(0);
          realtime_pub_->msg_.pid_d1[i] = talon_state_[i]->getPidfD(0);
          realtime_pub_->msg_.pid_f1[i] = talon_state_[i]->getPidfF(1);

          realtime_pub_->msg_.pid_p2[i] = talon_state_[i]->getPidfP(1);
          realtime_pub_->msg_.pid_i2[i] = talon_state_[i]->getPidfI(1);
          realtime_pub_->msg_.pid_d2[i] = talon_state_[i]->getPidfD(1);
          realtime_pub_->msg_.pid_f2[i] = talon_state_[i]->getPidfF(1);

          realtime_pub_->msg_.closed_loop_error[i] = talon_state_[i]->getClosedLoopError();
          realtime_pub_->msg_.forward_limit_switch[i] = talon_state_[i]->getFwdLimitSwitch();
          realtime_pub_->msg_.reverse_limit_switch[i] = talon_state_[i]->getRevLimitSwitch();
          //realtime_pub_->msg_.talon_mode[i] = talon_state_[i]->getTalonMode();
          int talonMode = talon_state_[i]->getTalonMode();
          switch(talonMode) {
            case -1:
                realtime_pub_->msg_.talon_mode[i] = "Uninitialized";
                break;
            case 0:
                realtime_pub_->msg_.talon_mode[i] = "Percent Vbus";
                break;
            case 1:
                realtime_pub_->msg_.talon_mode[i] = "Closed Loop Position";
                break;
            case 2:
                realtime_pub_->msg_.talon_mode[i] = "Closed Loop Speed";
                break;
            case 3:
                realtime_pub_->msg_.talon_mode[i] = "Closed Loop Current";
                break;
            case 4:
                realtime_pub_->msg_.talon_mode[i] = "Voltage";
                break;
            case 5:
                realtime_pub_->msg_.talon_mode[i] = "Follower";
                break;
            case 6:
                realtime_pub_->msg_.talon_mode[i] = "Motion Profile";
                break;
            case 7:
                realtime_pub_->msg_.talon_mode[i] = "Motion Magic";
                break;
            case 8:
                realtime_pub_->msg_.talon_mode[i] = "Last";
                break;
          }
        }
        realtime_pub_->unlockAndPublish();
      }
    }
  }

  void TalonStateController::stopping(const ros::Time& /*time*/)
  {}

  void TalonStateController::addExtraJoints(const ros::NodeHandle& nh,
  talon_state_controller::TalonState& msg)
  {

    // Preconditions
    XmlRpc::XmlRpcValue list;
    if (!nh.getParam("extra_joints", list))
    {
      ROS_DEBUG("No extra joints specification found.");
      return;
    }

    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Extra joints specification is not an array. Ignoring.");
      return;
    }

    for(int i = 0; i < list.size(); ++i)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << list[i].getType() <<
                         "'. Ignoring.");
        continue;
      }

      if (!list[i].hasMember("name"))
      {
        ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
        continue;
      }

      const std::string name = list[i]["name"];
      if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
      {
        ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
        continue;
      }

      const bool has_pos = list[i].hasMember("position");
      const bool has_vel = list[i].hasMember("velocity");
      const bool has_eff = list[i].hasMember("effort");

      const XmlRpc::XmlRpcValue::Type typeDouble = XmlRpc::XmlRpcValue::TypeDouble;
      if (has_pos && list[i]["position"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default position. Ignoring.");
        continue;
      }
      if (has_vel && list[i]["velocity"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default velocity. Ignoring.");
        continue;
      }
      if (has_eff && list[i]["effort"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default effort. Ignoring.");
        continue;
      }

      // State of extra joint
      const double pos = has_pos ? static_cast<double>(list[i]["position"]) : 0.0;
      const double vel = has_vel ? static_cast<double>(list[i]["velocity"]) : 0.0;
      const double eff = has_eff ? static_cast<double>(list[i]["effort"])   : 0.0;

      // Add extra joints to message
      msg.position.push_back(pos);
      msg.speed.push_back(vel);
      msg.output_voltage.push_back(eff);
      msg.can_id.push_back(0);
    }
  }

}

PLUGINLIB_EXPORT_CLASS( talon_state_controller::TalonStateController, controller_interface::ControllerBase)
