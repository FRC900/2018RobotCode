#include <dynamic_reconfigure/server.h>
#include <talon_controllers/TalonConfigConfig.h>

void callback(talon_controllers::TalonConfigConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f %f %f",
           config.p0,
           config.p1,
           config.i0,
           config.i1,
           config.d0,
           config.d1,
           config.f0,
           config.f1,
           config.izone0,
           config.izone1);

  // do nothing for now
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talon_reconfigure_node");
  dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig> srv;
  dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
