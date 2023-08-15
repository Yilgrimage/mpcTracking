# mpcTracking

MPC(Model predictive control) is widely used in robotics. this work uses MPC for path tracking of a two-wheeled differential cart in a simulation environment,and it is made to adapt to any path.

this work is based on turtle3(which support simulation sources) and osqp-eigen. Therefore, for you to be able to use this work correctly, please note the following points:

- it only work on ubuntu20.04, with ros-noetic
* ensure download all submoudles correctly
+ ensure your osqp-eigen lib is installed properly
- check all the path(maybe noetic path,lib path and so on)

## How it works? 
task1 is a simple work to test your env is right. It is implemented to allow the cart model to travel to a set position, which is, of course, entered by the keyboard,with a simple pid controller. Just run it to test.
        $roslaunch task1_pkg turtle3_test.launch
Obviously, task2 is the "protagonists", it

## How to use it?


There is currently no launch file written to start all nodes, you may need to do this manually, follow these steps：
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

## How to redevelop it?

For simple redesign, you only need pay attention to creatPath() in control_node.cpp, setWeightMatrices() and setInequalityConstraints() in mpc.cpp. it is very easy to resetting the path and adjusting the parameters of the controller.





