# Self-Balancing-Robot-Python-Simulator

 Here is presented a source code to simulate a self balancing two wheeled robot with pyhton and using OpenGL to display the simulated robot.

This simulator has three files:

    ibalancingbot.py that provides a class IBallancingBot that deals with the mathematical model of the robot and with the resolution of the differencial equations.

    pid.py that provides a PID class to be able to control the robot.

    main.py that uses OpenGL to display the robot and provides the menu to use the simulation

## iballancingbot.py files

This file provides the IBalancingBot class. This class contains a mathematical model for the self balancing robot. This model is extract from the article 'A Comparison of Controllers for Balancing Two wheeled Inverted Pendulum Robot' available here.
The IBalancingBot class uses a Runge-Kutta approach to compute the differential equations of the robot's dynamic.

## pid.py file

This file provides a simple discrete PID class that is used to control the robot in the simulation. This implementation was found here. 

## main.py file

This is the main file of the simulator. It uses OpenGL to display the state the IBalancingBot and uses PID to control the robot. It uses 3 PIDs : one for the pendulum angle, one for the linear speed and one for the angular rotation speed. The OpenGL part is based on a Jean-Baptiste Fasquel file (http://perso-laris.univ-angers.fr/~fasquel). 
