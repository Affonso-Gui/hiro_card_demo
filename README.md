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

#### Run from the Scratch web interface:
1. `roslaunch hiro_card_demo start.launch`
1. Open http://scratch3-ros.jsk.imi.i.u-tokyo.ac.jp
1. Add the ROS extension from the bottom left button on the screen and input the master URI (*localhost* or *hirovision*)
1. From the File menu load https://github.com/Affonso-Gui/hiro_card_demo/blob/master/src/card_demo_interface.sb3
1. Place and flip the cards as wanted and press Go to move the robot


#### Run from the console:
```shell
roscd hiro_card_demo/src
roseus main.l
```
