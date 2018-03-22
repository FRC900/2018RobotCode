/zebROS_ws/devel/lib/rosbag_scripts/rosbag_scripts_node $1
grep -Rl whatever temp_file.txt
rm temp_file.txt
#first, input bag
#second, run search_bag.cpp. it creates a temp file with all match_data text.
#then, search that file for actual data. 
#if returns true, rename the bag to fit the match data. if not, do not rename the bag
