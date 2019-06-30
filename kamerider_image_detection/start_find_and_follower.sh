#!/bin/sh

echo "start finding person"
gnome-terminal -x bash -c "rosrun kamerider_image_detection person_detection"

sleep 1

echo "start follow-me"
gnome-terminal -x bash -c "roslaunch turtlebot_follower my_follwer.launch"
sleep 1

