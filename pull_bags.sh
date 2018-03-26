JETSON_ADDR=10.9.0.8

ssh $JETSON_ADDR "cd /mnt/900_2/ && \
	./search_bag.sh /mnt/900_2/_2018*"

scp ubuntu@10.9.0.8:/mnt/900_2/match* .
