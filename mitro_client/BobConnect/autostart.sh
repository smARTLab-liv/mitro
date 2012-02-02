touch ~/autostart.test

cd /home/swarmlab/ros/mitro_client/BobConnect/experimental

source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=/home/swarmlab/ros:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://bob:11311 

DONE=0
while [ $DONE -lt 1 ] ; do
	echo waiting for network connection...
	ping -c1 root >/dev/null 2>&1
	if [ $? -eq 0 ]
	then
		roslaunch --pid=/home/swarmlab/mitro_bringup.pid mitro_bringup acer.launch &
		sleep 8 && sh runserver.sh 50000 /dev/video1 /dev/video2 &
		skype 
		DONE=1
	fi
done
