#also the organization is horrible. TODO please please fix

source /opt/ros/kinetic/setup.bash

IFS='
'

count=1

if [ $# -eq 0 ]
then
	echo Input a bag file
fi

for var in "$@"
do
	echo For bag file ${var}:

	if [[ $var == *bag.active ]]
	then 
		echo Reindexing
		rosbag reindex $var 
	fi

	if [[ $var == *.orig.active ]]
	then
		echo orig.active found -- skipping
		continue
	fi

	echo Writing data to file
	$HOME/2018RobotCode/zebROS_ws/devel/lib/rosbag_scripts/rosbag_scripts_node $var
	filename=$(basename $var)

	any_data=$(rosbag info $var | grep /frcrobot/match_data)
	if [ ! -z "$any_data" -a "$any_data" != " " ] 
	then
		echo This has match data
		matchNumber=$(grep -m1 matchNumber temp_file.txt | cut -c14-15)
		if [ $matchNumber = 0 ]
		then
			echo Match number is zero -- renaming
			mv $var $HOME/2018RobotCode/practice$(basename $var)
			continue
		fi

		alliance_data=$(grep this temp_file.txt)
		if [ ! -z "$alliance_data" -a "$alliance_data" != " " ]
		then 
			echo This has alliance data.
			bag_name=Match${matchNumber}
			echo Renaming bag file to ${bag_name}_${count}.bag
			cp $var $HOME/2018RobotCode/${bag_name}.bag
		else
			echo This does not have alliance data but match numer != 0 -- renaming to prematch${var}
			cp $var $HOME/2018RobotCode/prematch$(basename $var)
		fi
	else
		echo This does not have match data -- renaming to practice${var} 
		cp $var $HOME/2018RobotCode/practice$(basename $var)
	fi
	#rm temp_file.txt

done
