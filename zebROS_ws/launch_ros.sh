

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
    if [ $i = "-h" ] || [ $i = "--help" ]; then
        echo -e "\nUsage: ./launch_ros.sh \[OPTION...\] \[OPTION...\] ..."
        echo -e "launch_ros takes any number of arguments to launch different programs"
        echo -e "\nExamples:"
        echo -e "\t ./launch_ros.sh sim \t\t # Launches sim code"
        echo -e "\t ./launch_ros.sh sim ds \t # Launches sim code and RQT driver station"
        echo -e "\t ./launch_ros.sh rviz \t\t # Launches default rviz"
        echo -e "\t ./launch_ros.sh arm_viz.rviz \t # Launches arm_viz.rviz rviz file \n"
        echo -e "Possible options: \n"
        echo -e "\t sim \t-> launches ROS sim code"
        echo -e "\t robot \t-> launches robot code"
        echo -e "\t rviz \t-> launches default rviz"
        echo -e "\t ds \t-> launches RQT driver station sim"
        echo -e "\t any other argument is interpreted as a .rviz config file"
    elif [ $i = "rviz" ]; then
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
