Facial Expressions
==================

This is a catkin package that can control the iCub facial expression kit.

This kit is demonstrated here: https://www.youtube.com/watch?v=qsrs0e_9iX8.
Originally, it has been integrated into yarp (http://wiki.icub.org/yarp/). This package makes the expression kit usable in ROS. 

Dependencies
------------
This package has been tested on Ubuntu 12.04, ROS groovy and fuerte. It has also been tested on a Raspberry Pi running Raspian and ROS groovy which was installed as described here: http://wiki.ros.org/groovy/Installation/Raspbian. 

Furthermore, you have to install ace and yaml-cpp through

```bash
apt-get install libace-dev libyaml-cpp-dev
```

HowTo Install
-------------
The package can be installed like any other catkin package. Here we assume that we create a dedicated workspace for this one package. But it can be added to any other existing workspace.
```bash
mkdir ~/EMOTION
cd EMOTION
mkdir src
cd src
git clone this_package.git
cd ../../
catkin_make
source devel/setup.bash
```


HowTo Run
---------
Before you can control the facial expression kit, you have to make sure that after you plugged it in, a device gets created. Typically, you should find

```bash
ls /dev/ttyACM0
```
If the integer id is a different one, you have to also change the portname in the config file of the serial port: config/serialport.yaml.

Currently, there are two ways to test this package. The first one randomly cycles through the available expressions that are defined in config/emotions.yaml.

``` bash
roscd facial_expressions
rosrun facial_expressions random_emotion config/serialport.yaml config/emotions.yaml
```

The second option is to launch a subscriber that listens on the topic "expression" for incoming messages containing an emotion that is supposed to be visualized with the LEDs. 
This is most likely the node one would have running in normal operation.
```bash
roslaunch facial_expressions mood_manager.launch
```

After this node has been launched, it can be pinged with the following test node that cycles between happy and sad.
```bash
rosrun facial_expressions ping_exp_switch
```
