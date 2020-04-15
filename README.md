# Jetbot ROS Web Console

A Web Console for Nvidia Jetbot based on ROS Web Tools.
If is the first time you use this repo, execute all the following command, otherwise go to the **Run it** capter.

Poweron the Jetbot, and connect to in via SSH, execute the following command to install the ROS dependencies to use this package:

```shell
sudo apt-get install ros-melodic-roswww ros-melodic-rosbridge
```

Go in the Source folder of your workspace and clone this repository:

```shell
cd ~/workspace/catkin_ws/src
git clone https://github.com/bobboteck/jetbot_ros_webconsole.git
```

Use catkin_make to build the package:

```shell
cd ../           # or alternatively use: cd ~/workspace/catkin_ws
catkin_make
```

Check if the installation process went well, with the command:

```shell
rospack find jetbot_ros
```

And wait for the expected output:

```shell
/home/<USER>/workspace/catkin_ws/src/jetbot_ros_webconsole
```

## Run it

If you Jetbot is just up and running and you are connented in SSH, start ROS with command:

```shell
roscore
```

Now open a new terminal and execute this ROS command

```shell
roslaunch jetbot_ros_webconsole jetbot_ros_webconsole.launch
```

Good, now open your browser at the url:

```url
http://<JETBOT_IP-OR-JETBOT_HOSTNAME>:8080/jetbot_ros_webconsole
```

and see that ...
