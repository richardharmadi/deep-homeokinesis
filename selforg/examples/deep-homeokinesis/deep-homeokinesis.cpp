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

  StackInvertMotorNStep* main_controller = new StackInvertMotorNStep(10,3); // initialise with buffer size 10 and 2 layers
  InvertMotorNStep* controller0 = new InvertMotorNStep();
  //InvertMotorNStep* controller1 = new InvertMotorNStep();
  //InvertMotorNStep* controller2 = new InvertMotorNStep();
  main_controller->addLayer(*controller0);
  //main_controller->addLayer(*controller1);
  //main_controller->addLayer(*controller2);
  main_controller->init(2,2); // initialise with 2 motors and 2 sensors

  //controller->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
  //controller->print(stderr,0); // print parameters (see Configurable) to stderr

  // sensor and motor arrays (doubles*)
  double sensors[SNumber];
  double motors[MNumber];

  /*
  double sensor1[SNumber];
  double motor1[MNumber];

  double sensor2[SNumber];
  double motor2[MNumber];

  double sensor3[SNumber];
  double motor3[MNumber];
  */
  //double buffersensor[SNumber];// temp vector of inverted input
  //double buffermotor[MNumber]; // temp vector of inverted output

  //vector<double> nextsensor;
  //vector<double> nextmotor;

  memset(motors,0,sizeof(double)*MNumber);  // clear motors

  // the robot is here respresented by the function myrobot
  for(int i=0; i < 10; i++){
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

    /*
    for(int j=0;j<main_controller->getNLayer();j++){
      nextsensor.push_back(main_controller->getInvInputFromLayer(j));
      nextmotor.push_back(main_controller->getInvOutputFromLayer(j));
    }
    */

    /*
    sensor* sensor1 = main_controller->getInvInputFromLayer(0);
    motor* motor1 = main_controller->getInvOutputFromLayer(0);

    sensor* sensor2 = main_controller->getInvInputFromLayer(1);
    motor* motor2 = main_controller->getInvOutputFromLayer(1);

    sensor* sensor3 = main_controller->getInvInputFromLayer(2);
    motor* motor3 = main_controller->getInvOutputFromLayer(2);
    */

    /*
    cout << i << " Sensor X1: " << sensor1[0] << ", " << sensor1[1];
    cout << i << " Motor Y1: " << motor1[0] << ", " << motor1[1];

    cout << i << " Sensor X2: " << sensor2[0] << ", " << sensor2[1];
    cout << i << " Motor Y2: " << motor2[0] << ", " << motor2[1];


    cout << i << " Sensor X3: " << sensor3[0] << ", " << sensor3[1];
    cout << i << " Motor Y3: " << motor3[0] << ", " << motor3[1];
    */
    /*
    cout << i << " Sensor X1: " << *sensor1;
    cout << i << " Motor Y1: " << *motor1;
    cout << i << " Sensor X2: " << *sensor2;
    cout << i << " Motor Y2: " << *motor2;
    cout << i << " Sensor X3: " << *sensor3;
    cout << i << " Motor Y3 " << *motor3;
    */
  }
  delete[] main_controller;
  delete[] controller0;
  //delete controller1;
  //delete controller2;
  return 0;
}
