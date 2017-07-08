#include<assert.h>
#include<iostream>
#include<vector>
using namespace std;

#include <selforg/controller/stackinvertmotornstep.h>
#include <selforg/controller/invertmotornstep.h>
#include <selforg/controller/nstepwrapper.h>

const int MNumber = 2;
const int SNumber = 2;

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

  AbstractController* main_controller = new StackInvertMotorNStep(10,3); // initialise with buffer size 10 and 2 layers
  AbstractController* controller0 = new InvertMotorNStep(); // invertmotornstep object
  AbstractController* controller1 = new InvertMotorNStep();
  AbstractController* controller2 = new InvertMotorNStep();
  main_controller->addLayer(controller0);
  main_controller->addLayer(controller1);
  main_controller->addLayer(controller2);
  main_controller->init(2,2); // initialise with 2 motors and 2 sensors

  //controller->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
  //controller->print(stderr,0); // print parameters (see Configurable) to stderr

  // sensor and motor arrays (doubles*)
  double sensors[SNumber];
  double motors[MNumber];
  vector<sensor> sinv;// temp vector of inverted input
  vector<motor> minv; // temp vector of inverted output

  vector<vector<sensor>> nextsensor;
  vector<vector<motor>> nextmotor;

  memset(motors,0,sizeof(double)*MNumber);  // clear motors

  // the robot is here respresented by the function myrobot
  for(int i=0; i < 1000; i++){
    // call robot with motors and receive sensors 
    myrobot(sensors, SNumber, motors, MNumber);
    cout << i << " Sensor X0: " << sensors[0] << ", " << sensors[1];
    
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

    for(int i=0;i<main_controller.getNLayer;i++){
      nextsensor.push_back(main_controller.getInvInputFromLayer(i));
      nextmotor.push_back(main_controller.getInvOutputFromLayer(i));
    }

    cout << i << " Sensor X1: " << nextsensor[0].at(0) << ", " << nextsensor[0].at(1);
    cout << i << " Motor Y1: " << nextmotor[0].at(0) << ", " << nextmotor[0].at(1);

    cout << i << " Sensor X2: " << nextsensor[1].at(0) << ", " << nextsensor[1].at(1);
    cout << i << " Motor Y2: " << nextmotor[1].at(0) << ", " << nextmotor[1].at(1);


    cout << i << " Sensor X3: " << nextsensor[2].at(0) << ", " << nextsensor[2].at(1);
    cout << i << " Motor Y3: " << nextmotor[2].at(0) << ", " << nextmotor[2].at(1);
  }
  delete main_controller;
  delete controller0;
  delete controller1;
  delete controller2;
  return 0;
}
