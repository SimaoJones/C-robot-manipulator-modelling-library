#include <iostream>
#include "robot_manipulator_modeling.h"


using namespace std;

//Demonstration code of the library

int main()
{
    float l1 = 2.0;
    float l2 = 3.0;
    float l3 = 3.0;
    float lg = 1;

  //initialize the object that define for the robot
    robot_manipulator_modeling myrobot;

    //create three Links that correspond to three revolute joints
    myrobot.createLink(1,l1,90.0,0,0);
    myrobot.createLink(1,0,0.0,l2,0);
    myrobot.createLink(1,0,0.0,l3,0);
    myrobot.Add_End_Effector(lg);


    //show the Links defined
    myrobot.show_Links();
    return 0;
}
