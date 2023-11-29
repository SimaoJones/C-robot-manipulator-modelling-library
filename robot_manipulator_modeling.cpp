//Development of code for the library of robot manipulator modeling

#include <iostream>
#include <math.h>
#include <iomanip>
#include "robot_manipulator_modeling.h"

using namespace std;

#define PI 3.14159265

//Constructor for the robot
robot_manipulator_modeling::robot_manipulator_modeling(){
    N_Links = 1;
    base = nullptr;
    end_effector=false;
}


void robot_manipulator_modeling::createLink(bool revolute,float d, float a,float alpha, float offset){
    Link *newLink = new Link;
    Link *actual;
    newLink->next = nullptr;
    newLink->revolute = revolute;
    newLink->end_effector=false;


    /*float th;

    if(!revolute){
        th = 0;
    }else{
        //default value for theta
        th = PI/6;;
    } */

    newLink->a=a;
    newLink->alpha=alpha;
    newLink->offset =offset;
    newLink->d = d;


  newLink->T = new float*[4];
  for(int i = 0; i<4; i++){
    newLink->T[i] = new float[4];
  }


  for(int i = 0;i<4;i++){
      for(int j = 0;j<4;j++){
         newLink->T[i][j] =0.0;
      }

  }






    if(base==nullptr) base = newLink;
    else{
        actual = base;
        while(actual->next != nullptr)
            actual = actual->next;
        actual->next = newLink;
        N_Links++;
    }
}

//adding the end-effector of the robot
void robot_manipulator_modeling::Add_End_Effector(float l){




    l = lg;

    Link *newLink = new Link;
    Link *actual;
    newLink->next = nullptr;
    newLink->revolute = false;
    newLink->end_effector=true;


    float th;


        th = 0;
        newLink->a=0;
        newLink->alpha=0;
        newLink->offset =0;
        newLink->d = l;




  newLink->T = new float*[4];
  for(int i = 0; i<4; i++){
    newLink->T[i] = new float[4];
  }


   newLink->T[0][0] = cos(th+newLink->offset);
   newLink->T[0][1] = -cos(newLink->alpha)*sin(th+newLink->offset);
   newLink->T[0][2] = sin(newLink->alpha)*sin(th+newLink->offset);
   newLink->T[0][3] = newLink->a*cos(th+newLink->offset);

   newLink->T[1][0] = sin(th+newLink->offset);
   newLink->T[1][1] = cos(newLink->alpha)*cos(th+newLink->offset);
   newLink->T[1][2] = -sin(newLink->alpha)*cos(th+newLink->offset);
   newLink->T[1][3] = newLink->a*sin(th+newLink->offset);

   newLink->T[2][0] = 0.0000;
   newLink->T[2][1] = sin(newLink->alpha);
   newLink->T[2][2] = cos(newLink->alpha);
   newLink->T[2][3] = newLink->d;


   newLink->T[3][0] = 0.0000;
   newLink->T[3][1] = 0.0000;
   newLink->T[3][2] = 0.0000;
   newLink->T[3][3] = 1.0000;




//Check if the robot manipulator exists
    if(base==nullptr){
        cout<<"Error: there is no existing manipulator robot.Please use the method .createLink() to create the links of the robot"<<endl;
        return;
    }

//check if the Last Link of the robot is an end effector
   if(end_effector==true){
        cout<<"Error: the end effector was previously defined"<<endl;
   }else{
        actual = base;
        while(actual->next != nullptr)
            actual = actual->next;
        actual->next = newLink;
        N_Links++;
        end_effector = true;
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
        cout << "The robot is not defined, please use the method .createLink to create Links for the robot"<<endl;
    else{
        cout<<"Number of Links present in the robot "<<N_Links<<endl;
        while(actual !=nullptr){
            cout<<"Link number "<<n<<endl;
            if(actual->revolute == true && actual->end_effector==false){
                cout<<"Type of joint: revolute"<<endl;
            }else if(actual->revolute == false && actual->end_effector==false){
                cout<<"Type of joint: prismatic"<<endl;
            }else if(actual->revolute == false && actual->end_effector==true){
                cout<<"End Effector of the robot"<<endl;
            }

          /*  cout<<"Transformation matrix:  "<<endl;

            for(int i = 0; i<4;i++){
                cout<<"[ ";
                for(int j = 0; j<4;j++){
                    cout<<" "<<actual->T[i][j]<<setprecision(3);
                }
                cout<<" ]";
                cout<<endl;
            }*/
            actual = actual->next;
          n++;
        }
    }


}




//Compute the T_0_F matrix and show Transformation matrices that correspond to each link in an defined angle;
void robot_manipulator_modeling:: Foward_kinematics(void) const{
    int th =0;
    Link *actual = base;
    int n = 1;
    float **T_0_F;
    T_0_F = new float *[4];
    for(int i = 0; i<4; i++){
      T_0_F[i] = new float[4];
    }

    if(base==nullptr)
        cout << "The robot is not defined, please use the method .createLink to create Links for the robot"<<endl;
    else{
        while(actual !=nullptr){

            //check if the link is an end effector
            if(actual->end_effector){
                th = 0;
                cout<<"End effector";
                cout<<"\n";
            }else if(!actual->revolute && !actual->end_effector){
                th = 0;
                cout <<"write the value of d to the link number "<<n<<endl;
                cin>>actual->d;

            }else{
            cout <<"write the value of th angle to the link number "<<n<<" in degrees"<<endl;
            cin>>th;
            }
            th = th*(PI/180);
            actual->T[0][0] = cos(th+actual->offset);
            actual->T[0][1] = -cos(actual->alpha)*sin(th+actual->offset);
            actual->T[0][2] = sin(actual->alpha)*sin(th+actual->offset);
            actual->T[0][3] = actual->a*cos(th+actual->offset);

            actual->T[1][0] = sin(th+actual->offset);
            actual->T[1][1] = cos(actual->alpha)*cos(th+actual->offset);
            actual->T[1][2] = -sin(actual->alpha)*cos(th+actual->offset);
            actual->T[1][3] = actual->a*sin(th+actual->offset);

            actual->T[2][0] = 0.0000;
            actual->T[2][1] = sin(actual->alpha);
            actual->T[2][2] = cos(actual->alpha);
            actual->T[2][3] = actual->d;


            actual->T[3][0] = 0.0000;
            actual->T[3][1] = 0.0000;
            actual->T[3][2] = 0.0000;
            actual->T[3][3] = 1.0000;

            cout<<'\n';
            for(int i = 0; i<4;i++){
                cout<<"[ ";
                for(int j = 0; j<4;j++){
                    cout<<" "<<actual->T[i][j]<<setprecision(3);
                }
                cout<<" ]";
                cout<<endl;
            }

            cout<<"\n";


            actual = actual->next;
            n++;
        }
     }
}

//Obtain the wrist coordinates

/*float* robot_manipulator_modeling::Wrist_coordinates()const{
    float wrist_coord[3];
   //check presence of end-effector
    if(end_effector==false){
        cout<<"Error: The robot must have an end effector to obtain the wrist coordinates!<<endl";
        return 0;
    }

}*/







