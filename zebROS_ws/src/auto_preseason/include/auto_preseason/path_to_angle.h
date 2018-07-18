#include "auto_preseason/auto_loop.h"
#include "talon_state_controller/TalonState.h"

bool turn_to_angle_srv(auto_preseason::Angle::Request &req, const auto_preseason::Angle::Response &/*res*/);
bool snap_to_angle_srv(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/);
