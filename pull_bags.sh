JETSON_ADDR=10.9.0.8
ip=`/bin/hostname -I | tr -d ' ' | tr -d '\n'`

mkdir ~/bagfiles_from_jetson/
name='whoami'
at='@'
directory=':/home/'

ssh $JETSON_ADDR "cd 2018RobotCode && \
	./search_bag.sh /mnt/900_2/_2018* && \
	scp match* $name$at$ip$directory$name"
