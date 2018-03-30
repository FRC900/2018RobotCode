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
		alliance_data=$(grep this temp_file.txt)
		if [ ! -z "$alliance_data" -a "$alliance_data" != " " ]
		then 
			echo This has alliance data.
			matchNumber=$(grep -m1 matchNumber temp_file.txt | cut -c14-15)
			if [ $matchNumber = 0 ]
			then
				echo Match number is zero -- renaming
				mv $var $HOME/2018RobotCode/match0$(basename $var)
				continue
			fi
			bag_name=Match${matchNumber}
			if [ -e ${bag_name}*.bag ]
			then 
				echo This match already has a bag file -- merging to ${bag_name}_${count}.bag
				~/2018RobotCode/zebROS_ws/src/rosbag_scripts/scripts/merge_bagfiles.py ${bag_name}_${count}.bag $var ${bag_name}*.bag -v
				rm ${bag_name}_$(( $count - 1)).bag
				rm ${bag_name}.bag
				count=$(( $count + 1 ))
			else 
				echo Renaming bag file to ${bag_name}.bag
				cp $var $HOME/2018RobotCode/${bag_name}.bag
			fi
		else
			echo This does not have alliance data -- renaming to prematch${var}
			cp $var $HOME/2018RobotCode/prematch$(basename $var)
		fi
	else
		echo This does not have match data -- renaming to practice${var} 
		cp $var $HOME/2018RobotCode/practice$(basename $var)
	fi
	#rm temp_file.txt

done
