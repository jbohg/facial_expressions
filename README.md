facial_expressions
==================

This is a catkin package that can control the iCub facial expression kit.

This kit is demonstrated here: https://www.youtube.com/watch?v=qsrs0e_9iX8
Originally, it has been integrated into yarp (http://wiki.icub.org/yarp/). This package makes the expression kit usable in ROS. 

Dependencies
============
This package has been tested on Ubuntu 12.04, ROS groovy and fuerte. It has also been tested on a Raspberry Pi running Raspian and ROS groovy which was installed as described here: http://wiki.ros.org/groovy/Installation/Raspbian. 

Furthermore, you have to install ace and yaml-cpp through

```bash
apt-get install libace-dev libyaml-cpp-dev
```