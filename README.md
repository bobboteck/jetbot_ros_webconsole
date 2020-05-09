# Jetbot ROS Web Console

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/7f0b1a9e9cce4032a92a886f98df28c9)](https://app.codacy.com/manual/bobboteck/jetbot_ros_webconsole?utm_source=github.com&utm_medium=referral&utm_content=bobboteck/jetbot_ros_webconsole&utm_campaign=Badge_Grade_Settings)

## About

**Author: [Roberto D'Amico](http://bobboteck.github.io)**

A **Web Console for Nvidia Jetbot** based on *ROS Web Tools*.

If is the first time you use this repo, execute all the following command, ootherwise check if there are code updates with the git command, in case you download them, and after go to the **Run it** section.

## How to use it

Poweron the Jetbot, and connect to in via SSH, execute the following command to install the ROS dependencies to use this package:

```shell
sudo apt-get install ros-melodic-roswww ros-melodic-rosbridge-server
```

**The latest version of code** need the [Jetson stats](https://github.com/rbonghi/jetson_stats) installed, to install it simply use the command:

```shell
sudo -H pip install -U jetson-stats
```

and after clone the report of wrapper in the your source folder

```shell
cd ~/workspace/catkin_ws/src
git clone https://github.com/rbonghi/ros_jetson_stats.git
```

Now in the Source folder of your workspace and clone this repository, for rexample:

```shell
cd ~/workspace/catkin_ws/src # Only if you not in this folder
git clone https://github.com/bobboteck/jetbot_ros_webconsole.git
```

Go back in the catkin_ws and use catkin_make to build the package:

```shell
cd ../           # or alternatively use: cd ~/workspace/catkin_ws
catkin_make
```

Check if the installation process went well, with the command:

```shell
rospack find jetbot_ros_webconsole
```

And wait for the expected output:

```shell
/home/<USER>/workspace/catkin_ws/src/jetbot_ros_webconsole
```

> **Note**: the folders I have indicated are not abbligatory those to be used, if necessary modify the scripts according to the folders of your project

## Run it

If you Jetbot is just up and running and you are connented in SSH, start the console with command:

```shell
roslaunch jetbot_ros_webconsole jetbot_ros_webconsole.launch
```

Good, now open a browser on a device connected in the same network of the Robot, and go to the url:

```url
http://<JETBOT_IP-OR-JETBOT_HOSTNAME>:8080/jetbot_ros_webconsole
```

and see that ...

## Feature

List of feature implemented, this list is in continuos evolution:

* Connection and control of it
* CPU information
* Move Robot with JoyStick
* Send message on display

## Contribute

To report BUG or request new Features, you can use GitHub's [ISSUE](https://github.com/bobboteck/jetbot_ros_webconsole/issues) system related to this project.
