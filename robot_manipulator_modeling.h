#ifndef ROBOT_MANIPULATOR_MODELING_H
#define ROBOT_MANIPULATOR_MODELING_H
#include <iostream>
using namespace std;

/*Robotic Manipulator Kinematic Modeling Library
 Developer:Simão Franco Jones, Student of Master in Electrical and Computer engineering in the branch of Robotics, Control and Artificial Intelligence
 Institution: DEEC (Departamento de engenharia eletrotécnica e de Computadores, FCTUC (Faculdade de Ciências e Tecnologias da Universidade de Coimbra),polo 2

 About:
 This library aims to help understanding the concepts that are used to model robotic manipulators by using the the Denavith Hartenberg foward kinematic method to obtain the model.
 The modeling of the robot manipulator is based on a linked list data structure.
 The linked list represents the Robot manipulator, each node present in the list represents each link that we define in the robot accordingly to the Denavith-Hartenberg method.
 The object is initialized with N_Links = 1, which it means that initially we have only the base (head of the linked list)
 The robot is built by a method that creates each Link.

 Example: RRR planar robot

            ------ +++++++++++++
           |      |            +
           |      |            +
          + ------++++++++++++++
         +     +
        +     +
      ------+
     |      |
     |      |
      ------
      +    +
      +    +
      ------
     |      |
  +++|      |++++
  +    ----     +
  +             +
  +++++++++++++++


  Linked list created


  |---|   |---------|--|    |----------|--|   |---------|--|   |---------|--|
  |   |   |         |  |    |          |  |   |         |  |   |         |  |
  |   |-->|         |  |--->|          |  |-->|         |  |-->|         |  |-->nullptr
  |---|   |---------|--|    |----------|--|   |---------|--|   |---------|--|

  Base       Link1              Link2             Link3         end_effector



*/
//class that defines each link that describes the robot
class Link{
   public:
    bool revolute; //flag that defines if the i-1 joint present in the link is revolute or prismatic
    //revolute = 1 -> revolute joint
    //revolute = 0 -> prismatic joint

    bool end_effector; //flag that indicates if the link is an end effector
    float **T; //Transformation matrix T_i-1_i that describres the Link between the joints i-1 and i

    Link *next;

};


class robot_manipulator_modeling{

    Link *base; // defining the base of the robot manipulator (linked list head)
    int N_Links; //defining the number of Links that the robot has
    float lg;//distance between the last joint and end effector
     bool end_effector; //flag for presence of end_effector;



 public:
    robot_manipulator_modeling(void); //defining the constructor for the robot manipulator
    ~robot_manipulator_modeling(void); //defining the destructor for the

    void createLink(bool revolute,float d, float alpha, float a, float offset);

    void Add_End_Effector(float l);

    void show_Links(void) const;

    void Foward_kinematics(void) const;

    float Wrist_coordinates(void) const;



};


#endif // ROBOT_MANIPULATOR_MODELING_H
