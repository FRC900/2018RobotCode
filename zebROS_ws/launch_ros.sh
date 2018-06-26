

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
launch_rqt_plot()
{
    rosrun rqt_plot rqt_plot &
}
launch_rqt_graph()
{
    rosrun rqt_graph rqt_graph &
}
launch_rqt_gui()
{
    rosrun rqt_gui rqt_gui &
}
launch_rqt_console()
{
    rosrun rqt_console rqt_console &
}
launch_visualizer()
{
    python ~/2018RobotCode/zebROS_ws/src/visualize_profile/scripts/visualizer.py &
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
        echo -e "\t sim \t\t-> launches ROS sim code"
        echo -e "\t robot \t\t-> launches robot code"
        echo -e "\t rviz \t\t-> launches default rviz"
        echo -e "\t rqt_plot \t-> launches rqt_plot"
        echo -e "\t rqt_graph \t-> launches rqt_graph"
        echo -e "\t rqt_console \t-> launches rqt_console"
        echo -e "\t rqt_gui \t-> launches rqt_gui"
        echo -e "\t ds \t\t-> launches RQT driver station sim"
        echo -e "\t any other argument is interpreted as a .rviz config file"
    elif [ $i = "rviz" ]; then
        launch_rviz
    elif [ $i = "sim" ]; then
        run_sim=1
    elif [ $i = "robot" ]; then
        run_code=1
    elif [ $i = "ds" ]; then
        launch_driver_station
    elif [ $i = "rqt_plot" ]; then
        launch_rqt_plot
    elif [ $i = "rqt_graph" ]; then
        launch_rqt_graph
    elif [ $i = "rqt_console" ]; then
        launch_rqt_console
    elif [ $i = "rqt_gui" ]; then
        launch_rqt_gui
    elif [ $i = "visualize" ]; then
        launch_visualizer
    else
        launch_specific_rviz $i
    fi
done
if [ "$run_sim" -ne 0 ]; then
    launch_sim_code
elif [ "$run_code" -ne 0 ]; then
    launch_robot_code
fi
