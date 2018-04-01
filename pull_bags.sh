JETSON_ADDR=10.9.0.8

#ssh $JETSON_ADDR "/home/ubuntu/2018RobotCode/search_bag.sh /mnt/900_2/_2018-03-26*"

scp ubuntu@10.9.0.8:/mnt/900_2/_2018-03-* .
./search_bag.sh _2018-*

