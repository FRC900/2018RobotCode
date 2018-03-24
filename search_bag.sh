source /opt/ros/kinetic/setup.bash

echo check for reindex
if [[ $1 == *.active ]]
then 
	echo Reindexing
	rosbag reindex $1 
fi

match='match'
bag='.bag'

echo Writing data to file
zebROS_ws/devel/lib/rosbag_scripts/rosbag_scripts_node $1

echo Is this a match?
frame_id=$(grep 'frame_id' temp_file.txt)
if [[ -n frame_id ]]
then
	echo This has match data
	matchNumber=$(sed -n 10p temp_file.txt)
	bag_name=$match$matchNumber$bag
	if [[ -e $bag_name ]]
	then 
		echo This match already has a bag file... merging
		#cat $1 >> $bag_name 
	else 
		echo Renaming bag file
		cp $1 $bag_name
	fi
else
	echo This does not have match data.
fi


#rm temp_file.txt


#first, input bag
#second, run search_bag.cpp. it creates a temp file with all match_data text.
#then, search that file for actual data. 
#if returns true, rename the bag to fit the match data. if not, do not rename the bag
