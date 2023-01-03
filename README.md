## Hiro Card Demo

<p align="center">
  <img height="300" src="https://user-images.githubusercontent.com/20625381/78788917-678e3280-79e7-11ea-97c1-b84a2465a2ca.jpg">
</p>

#### Quick Install:
```shell
cd catkin_ws/src
git clone https://github.com/Affonso-Gui/hiro_card_demo.git
cd catkin_ws
catkin build hiro_card_demo
```

#### Requirements

This demo needs the rubber attachments shown below.
<p align="left">
  <img height="200" src="https://user-images.githubusercontent.com/20625381/210297979-a8e0980b-6a55-4295-bf5e-c697ee2bdf58.jpg">

It also relies on **ACCURATE JOINT CALIBRATION**. 
Make sure to always calibrate from the power-off pose, and try to recalibrate when not being able to grasp cards correctly.

#### Run from the Scratch web interface:
1. `roslaunch hiro_card_demo start.launch`
1. Open http://scratch3-ros.jsk.imi.i.u-tokyo.ac.jp
1. Add the ROS extension from the bottom left button on the screen and input the master URI (*localhost* or *hirovision*)
1. From the File menu load [card_demo_interface.sb3](https://github.com/Affonso-Gui/hiro_card_demo/blob/master/src/card_demo_interface.sb3)
1. Place and flip the cards as wanted and press Go to move the robot


#### Run from the console:
```shell
roscd hiro_card_demo/src
roseus main.l
```
