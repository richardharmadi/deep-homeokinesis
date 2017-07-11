#include<assert.h>
#include<iostream>
#include<vector>
#include <fstream>

using namespace std;

#include <selforg/controller/stackinvertmotornstep.h>
#include <selforg/controller/invertmotornstep.h>
#include <selforg/controller/nstepwrapper.h>

const int MNumber = 2;
const int SNumber = 2;
const int buffersize = 50;
ofstream myfile;

/** The robot control should go here
    @param sensors list of sensor values (to be written) (doubles)
    @param sensornumber length of the sensors vector
    @param motors list of motor values (doubles) (to send to the robot)
    @param motornumber length of the motors vector

 */
void myrobot(double* sensors, int sensornumber, const double* motors, int motornumber){  
  assert(sensornumber >= 2 && motornumber >= 2); // check dimensions
  //the robot consits here just of a bit noise
  sensors[0]=motors[0]+(double(rand())/RAND_MAX-0.5)*0.3;
  sensors[1]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
}

int main(){
  myfile.open("~/example.csv");
  StackInvertMotorNStep* main_controller = new StackInvertMotorNStep(buffersize,3); // initialise with buffer size 50 and 2 layers
  InvertMotorNStep* controller0 = new InvertMotorNStep();
  InvertMotorNStep* controller1 = new InvertMotorNStep();
  //InvertMotorNStep* controller2 = new InvertMotorNStep();
  main_controller->addLayer(controller0);
  main_controller->addLayer(controller1);
  //main_controller->addLayer(*controller2);
  main_controller->init(2,2); // initialise with 2 motors and 2 sensors

  //controller->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
  //controller->print(stderr,0); // print parameters (see Configurable) to stderr

  // sensor and motor arrays (doubles*)
  double sensors[SNumber];
  double motors[MNumber];

  vector<vector<sensor>> nextsensor;
  vector<vector<motor>> nextmotor;

  memset(motors,0,sizeof(double)*MNumber);  // clear motors

  // the robot is here respresented by the function myrobot
  for(int i=0; i < 100; i++){
    // call robot with motors and receive sensors 
    myrobot(sensors, SNumber, motors, MNumber);
    cout << i << " Sensor X0: " << sensors[0] << ", " << sensors[1] << endl;
    myfile << "Sensor X0,Motor Y0,Sensor X1,Motor Y1";
    myfile << sensors[0] << ",";
    /*
    // print some internal parameters of the controller
    list<Inspectable::iparamval> list_ = controller1->getInternalParams();  
    vector<Inspectable::iparamval> v(list_.begin(), list_.end());  
    cout << i << " C00: " << v[4] << ", " <<" C01: " << v[5] << ", " <<
      " C10: " << v[6] << ", " <<" C11: " << v[7]  << endl;
    */
 
    // call controller with sensors and receive motors (both dimension 2)    
    main_controller->step(sensors, SNumber, motors, MNumber); 
    cout << i << " Motor Y0: " << motors[0] << ", " << motors[1] << endl;
    myfile << motors[0] << ",";

    if(i>buffersize){
      for(int j=0;j<main_controller->getNLayer()-1;j++){
        //main_controller->getInvInputFromLayer(j);
        //main_controller->getAvgOutputFromLayer(j);
        nextsensor.push_back(main_controller->getInvInputFromLayer(j)); 
        nextmotor.push_back(main_controller->getAvgOutputFromLayer(j));
      }
      cout << i << " Sensor X1: " << nextsensor[0][0] << ", " << nextsensor[0][1] << endl;
      cout << i << " Motor Y1: " << nextmotor[0][0] << ", " << nextmotor[0][1] << endl;
      myfile << nextsensor[0][0] << "," << nextmotor[0][0] << ",\n";
    }

   
    /*
      cout << i << " Sensor X2: " << nextsensor[1][0] << ", " << nextsensor[1][1];
      cout << i << " Motor Y2: " << nextmotor[1][0] << ", " << nextmotor[1][1];


      cout << i << " Sensor X3: " << nextsensor[2][0] << ", " << nextsensor[2][1];
      cout << i << " Motor Y3: " << nextmotor[2][0] << ", " << nextmotor[2][1];
    */   
  }
  //cout << " Sensor X1: " << nextsensor[0][0] << ", " << nextsensor[0][1] << endl;
  //cout << " Motor Y1: " << nextmotor[0][0] << ", " << nextmotor[0][1] << endl;
  delete main_controller;
  delete controller0;
  delete controller1;
  //delete controller2;
  myfile.close();
  return 0;
}
