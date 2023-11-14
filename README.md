# C++-robot-manipulator-modelling-library
This library aims to help understanding the concepts that are used to model robotic manipulators by using the the Denavith Hartenberg foward kinematic method to obtain the model.  The modeling of the robot manipulator is based on a linked list data structure.


# Example RRR planar robot


Planar Robot model

![planar robot](https://github.com/SimaoJones/Cplusplus-robot-manipulator-modelling-library/assets/94750658/5a949793-7b66-4587-aefd-04c901dc99ca)

Linked List that models the planar robot

![linked list](https://github.com/SimaoJones/Cplusplus-robot-manipulator-modelling-library/assets/94750658/9dd9421f-007c-494d-8d8e-90b03e2335ff)




The Library is programmed in `` QT Creator ``

The Library consists in two classes: ``Link `` and ``robot_manipulator_modeling``

The ``Link`` class defines each Linked List node that represents the link of the robot.

The public attributes present in the ``Link class`` are:

 bool revolute (flag that indicates if the joint is prismatic or revolute)
 
 bool end_effector (flag that indicates if the Link is end effector)
 
 float **T (Transformation matrix of the Link)

 The ``robot_manipulator_modeling`` class is used to model our robotic manipulator
 
 The private attributes are:
 
 Link *base (define the base of the manipulator)
 
 int N_Links(number of Links that the robot has)
 
 float lg (distance between the last joint and end effector)
 
 bool end_effector (flag that defines the presence of end effector)

 The robot that is creating with the constructor that starts with the base and adds the Links to the robot using the public method .createLink() and adds the end-effector with the method .Add_End_Effector
 
 


