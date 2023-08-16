# mpcTracking
## Background
MPC(Model predictive control) is widely used in robotics. this work uses MPC for path tracking of a two-wheeled differential cart in a simulation environment, and it is made to adapt to any path.

this work is based on [turtle3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)(which support simulation sources) and [osqp-eigen](https://github.com/robotology/osqp-eigen), you may get more infomation in their website. Btw if you are new for MPC, you should learn it quickly in [Model Predictive Control](https://www.bilibili.com/video/BV1HQ4y1P7bJ/?share_source=copy_web&vd_source=9ccc920bc6359b6c70aeb81313354624)(but please note the difference between models), after that you may understand how it works. There is other site teach you how to [cast MPC to QP](https://robotology.github.io/osqp-eigen/md_pages_mpc.html) with osqp-eigen.

## Install
 for you to be able to use this work correctly, please note the following points:

- it only work on ubuntu20.04, with ros-noetic-fulldesktop.
* ensure download all submoudles correctly
+ ensure your osqp-eigen lib is installed properly
- check all the path(noetic path,lib path and so on)

It is easy to configuration environment, you can find comprehensive and correct installation guide in their Github Page or stackflow.



## Usage
### How to use it?
task1 is a simple work to test your env is right. It is implemented to allow the cart model to travel to a set position, which is, of course, entered by the keyboard,with a simple pid controller. Just run it to test.

        $roslaunch task1_pkg turtle3_test.launch
About task2, there is currently no launch file to start all nodes automatically for some reason, you may need to do this manually, follow these steps：
1. go to the main folder

        $ cd path/to/work
1. maybe you should re-build it
   
        $ catkin_make
1. import the model name, and start up Gazebo,you could try other model like waffle,but it means you should modify the code to support different robot model.
   
        $ export TURTLEBOT3_MODEL=burger
        $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
1. start up Rviz
   
        $ rviz -d ./src/task2_pkg/rviz/mpctracking.rviz
1. start mpc control node, and you may get some INFO in screen, and model will move.
   
        $rosrun task2_pkg control

### How to redevelop it?

For simple redesign, you only need pay attention to _creatPath()_ in _control_node.cpp_, _setWeightMatrices()_ and _setInequalityConstraints()_ in _mpc.cpp_. it is very easy to resetting the path and adjusting the parameters of the controller.

## Maintain

Yilgrimage




