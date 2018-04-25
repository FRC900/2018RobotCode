

launch_sim_code()
{
    roslaunch ros_control_boilerplate 2018_main_frcrobot.launch hw_or_sim:=sim
}
launch_robot_code()
{
    roslaunch controller_node controller_master.launch
}
launch_rviz()
{
    rosrun rviz rviz &
}
launch_specific_rviz()
{
    rosrun rviz rviz -d $1 &
}
launch_driver_station()
{
    rqt --standalone rqt_driver_station_sim &
}

run_sim=0
run_code=0

for i in $@; do
    if [ $i = "rviz" ]; then
        launch_rviz
    elif [ $i = "sim" ]; then
        run_sim=1
    elif [ $i = "robot" ]; then
        run_code=1
    elif [ $i = "ds" ]; then
        launch_driver_station
    else
        launch_specific_rviz $i
    fi
done
if [ "$run_sim" -ne 0 ]; then
    launch_sim_code
elif [ "$run_code" -ne 0 ]; then
    launch_robot_code
fi
