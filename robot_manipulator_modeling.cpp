//Development of code for the library of robot manipulator modeling

#include <iostream>
#include <math.h>
#include "robot_manipulator_modeling.h"
using namespace std;


//Constructor for the robot
robot_manipulator_modeling::robot_manipulator_modeling(){
    N_Links = 1;
    base = nullptr;
}


void robot_manipulator_modeling::createLink(bool revolute,float d, float alpha, float a, float offset){
    Link *newLink = new Link;
    Link *actual;
    newLink->next = nullptr;
    newLink->revolute = revolute;
    newLink->end_effector=false;


    float th;

    if(!revolute){
        th = 0;
    }else{
        //default value for theta
        th = 20.0;
    }

  newLink->T = new float*[4];
  for(int i = 0; i<4; i++){
    newLink->T[i] = new float[4];
  }


   newLink->T[0][0] = cos(th+offset);
   newLink->T[0][1] = -cos(alpha)*sin(th+offset);
   newLink->T[0][2] = sin(alpha)*sin(th+offset);
   newLink->T[0][3] = a*cos(th+offset);

   newLink->T[1][0] = sin(th+offset);
   newLink->T[1][1] = cos(alpha)*cos(th+offset);
   newLink->T[1][2] = -sin(alpha)*cos(th+offset);
   newLink->T[1][3] = a*sin(th+offset);

   newLink->T[2][0] = 0;
   newLink->T[2][1] = sin(alpha);
   newLink->T[2][2] = cos(alpha);
   newLink->T[2][3] = d;


   newLink->T[3][0] = 0;
   newLink->T[3][1] = 0;
   newLink->T[3][2] = 0;
   newLink->T[3][3] = 1;






    if(base==nullptr) base = newLink;
    else{
        actual = base;
        while(actual->next != nullptr)
            actual = actual->next;
        actual->next = newLink;
        N_Links++;
    }
}


//destructor for the robot
robot_manipulator_modeling:: ~robot_manipulator_modeling(){
 Link *actual,*next;
 actual = base;
 while(actual != nullptr){
     next = actual->next;
     delete actual;
     actual = next;
 }
}



//Display the Links
void robot_manipulator_modeling::show_Links() const{
    Link *actual = base;
    int n = 1;
    if(base==nullptr)
        cout << "The robot is not defined, please use the method .createLink to create Links for the robot";
    else{
        cout<<"Number of Links present in the robot "<<N_Links<<endl;
        while(actual !=nullptr){
            cout<<"Link number "<<n<<endl;
            if(actual->revolute){
                cout<<"Type of joint: revolute"<<endl;
            }else{
                cout<<"Type of joint: prismatic"<<endl;
            }

            cout<<"Transformation matrix:  "<<endl;

            for(int i = 0; i<4;i++){
                cout<<"[ ";
                for(int j = 0; j<4;j++){
                    cout<<" "<<actual->T[i][j];
                }
                cout<<" ]";
                cout<<endl;
            }
            actual = actual->next;
          n++;
        }
    }


}
