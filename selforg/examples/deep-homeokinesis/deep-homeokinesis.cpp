#include<assert.h>
#include<iostream>
#include<math.h>
#include<vector>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

#include <selforg/controller/stackinvertmotornstep.h>
#include <selforg/controller/invertmotornstep.h>
#include <selforg/controller/nstepwrapper.h>

const int MNumber = 2;
const int SNumber = 2;
const int buffersize = 50;
ofstream inputvalue,outputvalue;
stringstream ss;

#define PI 3.14159265

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

void sinerobot(double param, double* sensors, int sensornumber, const double* motors, int motornumber){
  assert(sensornumber >= 2 && motornumber >= 2); // check dimensions
  //the robot consits here just of a bit noise
  sensors[0]=sin(param*PI/180);
  sensors[1]=motors[1]+(double(rand())/RAND_MAX-0.5)*0.3;
}

void zerorobot(double* sensors, int sensornumber, const double* motors, int motornumber){
  assert(sensornumber >= 2 && motornumber >= 2); // check dimensions
  //the robot consits here just of a bit noise
  sensors[0]=(double(rand())/RAND_MAX-0.5)*0.0;
  sensors[1]=(double(rand())/RAND_MAX-0.5)*0.0;
}

void noiserobot(double* sensors, int sensornumber, const double* motors, int motornumber){  
  assert(sensornumber >= 2 && motornumber >= 2); // check dimensions
  //the robot consits here just of a bit noise
  sensors[0]=(double(rand())/RAND_MAX-0.5)*0.3;
  sensors[1]=(double(rand())/RAND_MAX-0.5)*0.3;
}
int main(int argc, char* argv[]){
  if(argc < 4){
    std::cerr << "Usage: " << argv[0] << " NUMBER_LAYER " << " INPUT_SIGNAL" << " MODEL" << std::endl;
    /* "Usage messages" are a conventional way of telling the user
     * how to run a program if they enter the command incorrectly.
     */
    return 1;
  }

  stringstream ss_input;
  stringstream ss_output;
  ss_input << argv[3] << "_" << argv[2] << "_" << argv[1] <<"layer_input.csv";
  ss_output << argv[3] << "_" << argv[2] << "_" << argv[1] <<"layer_output.csv";

  string filename_input = ss_input.str();
  string filename_output = ss_output.str();

  inputvalue.open(filename_input);
  outputvalue.open(filename_output);
  //---------- 1 layer (baseline) -----------//
  // inputvalue.open("top-down_1layer_input.csv");
  // outputvalue.open("top-down_zero1layer_output.csv");
  //---------- 2 layers ---------------------//
  // inputvalue.open("top-down_zero2layers_input.csv");
  // outputvalue.open("top-down_zero2layers_output.csv");

  // ---------- 3 layers ---------------------//
  // inputvalue.open("top-down_noise3layers_input_rateA001-Ce3.csv");
  // outputvalue.open("top-down_noise3layers_outputrateA001-Ce3.csv");
  
  // ---------- 5 layers --------------------//
  // inputvalue.open("top-down_5layers_input.csv");
  // outputvalue.open("top-down_5layers_output.csv");

  StackInvertMotorNStep* main_controller = new StackInvertMotorNStep(buffersize,10); // initialise with buffer size 50 and 2 layers
  InvertMotorNStep* controller0 = new InvertMotorNStep();
  InvertMotorNStep* controller1 = new InvertMotorNStep();
  InvertMotorNStep* controller2 = new InvertMotorNStep();
  InvertMotorNStep* controller3 = new InvertMotorNStep();
  InvertMotorNStep* controller4 = new InvertMotorNStep();
  if(strcmp(argv[1],"1")==0){
    // ---------- 1 layer ----------------------//
    main_controller->addLayer(controller0);
  }else if(strcmp(argv[1],"2")==0){
    // ---------- 2 layers ---------------------//
    main_controller->addLayer(controller0);
    main_controller->addLayer(controller1);
  }else if(strcmp(argv[1],"3")==0){
    // ---------- 3 layers ---------------------//
    main_controller->addLayer(controller0);
    main_controller->addLayer(controller1);
    main_controller->addLayer(controller2);

   //controller0->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
    //controller1->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
    //controller2->setParam("epsA",0.01); // set parameter epsA (learning rate for Model A)
    
    //controller0->setParam("epsC",1.0e-3); // set parameter epsA (learning rate for Model A)
    //controller1->setParam("epsC",1.0e-3); // set parameter epsA (learning rate for Model A)
    //controller2->setParam("epsC",1.0e-3); // set parameter epsA (learning rate for Model A)
  }else if(strcmp(argv[1],"5")==0){
    // ---------- 5 layers ---------------------//
    main_controller->addLayer(controller0);
    main_controller->addLayer(controller1);
    main_controller->addLayer(controller2); 
    main_controller->addLayer(controller3);
    main_controller->addLayer(controller4);
  }
  
  main_controller->init(2,2); // initialise with 2 motors and 2 sensors
  //controller->print(stderr,0); // print parameters (see Configurable) to stderr

  // ----- sensor and motor arrays (doubles*)
  double sensors[SNumber];
  double motors[MNumber];

  vector<vector<sensor>> nextsensor;
  vector<vector<motor>> nextmotor;

  memset(motors,0,sizeof(double)*MNumber);  // clear motors
  inputvalue << "Sensor X0,Motor Y0,\n";

  if(strcmp(argv[1],"2")==0){
    outputvalue << "Sensor X1,Motor Y1,\n";
  }else if(strcmp(argv[1],"3")==0){
    outputvalue << "Sensor X1,Motor Y1,Sensor X2,Motor Y2,\n";
  }else if(strcmp(argv[1],"5")==0){
    outputvalue << "Sensor X1,Motor Y1,Sensor X2,Motor Y2,Sensor X3,Motor Y3, Sensor X4,Motor Y4,\n";
  }
  // ---------- 10 layers ---------------------//
  //outputvalue << "Sensor X1,Motor Y1,Sensor X2,Motor Y2,Sensor X3,Motor Y3, Sensor X4,Motor Y4,Sensor X5,Motor Y5,Sensor X6,Motor Y6,Sensor X7,Motor Y7,Sensor X8,Motor Y8,Sensor X9,Motor Y9\n";

  // the robot is here respresented by the function myrobot
  for(int i=0; i < 1000; i++){
    // call robot with motors and receive sensors
    if(strcmp(argv[2],"zero")==0){
      zerorobot(sensors,SNumber,motors,MNumber);
    }else if(strcmp(argv[2],"noise")==0){
      noiserobot(sensors,SNumber,motors,MNumber);
    }else if(strcmp(argv[2],"sinewave")==0){
      sinerobot(i,sensors,SNumber,motors,MNumber);
    }else if(strcmp(argv[2],"normal")==0){
      myrobot(sensors,SNumber,motors,MNumber);
    }
    cout << i << " Sensor X0: " << sensors[0] << ", " << sensors[1] << endl;
    inputvalue << sensors[0] << ",";
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
    inputvalue << motors[0] << ",\n";

    if(i>buffersize){
      for(size_t j=0;j<main_controller->getNLayer()-1;j++){
        if((nextsensor.size()<=j) && (nextmotor.size()<=j)) { // if the vector size is less than number of controller
          nextsensor.push_back(main_controller->getInvInputFromLayer(j));   
          nextmotor.push_back(main_controller->getAvgOutputFromLayer(j));
        }else{
          nextsensor[j] = main_controller->getInvInputFromLayer(j);
          nextmotor[j] = main_controller->getAvgOutputFromLayer(j);
        }
      }  

      // cout << i << " Sensor X5: " << nextsensor[4][0] << ", " << nextsensor[4][1] << endl;
      // cout << i << " Motor Y5: " << nextmotor[4][0] << ", " << nextmotor[4][1] << endl;
      // cout << i << " Sensor X6: " << nextsensor[5][0] << ", " << nextsensor[5][1];
      // cout << i << " Motor Y6: " << nextmotor[5][0] << ", " << nextmotor[5][1];
      // cout << i << " Sensor X7: " << nextsensor[6][0] << ", " << nextsensor[6][1];
      // cout << i << " Motor Y7: " << nextmotor[6][0] << ", " << nextmotor[6][1];   
      // cout << i << " Sensor X8: " << nextsensor[7][0] << ", " << nextsensor[7][1];
      // cout << i << " Motor Y8: " << nextmotor[7][0] << ", " << nextmotor[7][1];   
      // cout << i << " Sensor X9: " << nextsensor[8][0] << ", " << nextsensor[8][1];
      // cout << i << " Motor Y9: " << nextmotor[8][0] << ", " << nextmotor[8][1];   
      if(strcmp(argv[1],"2")==0){
        cout << i << " Sensor X1: " << nextsensor[0][0] << ", " << nextsensor[0][1] << endl;
        cout << i << " Motor Y1: " << nextmotor[0][0] << ", " << nextmotor[0][1] << endl;
        outputvalue << nextsensor[0][0] << "," << nextmotor[0][0] << ",\n"; // for 2 layers purpose
      }else if(strcmp(argv[1],"3")==0){
        cout << i << " Sensor X1: " << nextsensor[0][0] << ", " << nextsensor[0][1] << endl;
        cout << i << " Motor Y1: " << nextmotor[0][0] << ", " << nextmotor[0][1] << endl;
        cout << i << " Sensor X2: " << nextsensor[1][0] << ", " << nextsensor[1][1] << endl;
        cout << i << " Motor Y2: " << nextmotor[1][0] << ", " << nextmotor[1][1] << endl;
        outputvalue << nextsensor[0][0] << "," << nextmotor[0][0] << ","; // for 3 layers purpose
        outputvalue << nextsensor[1][0] << "," << nextmotor[1][0] << ",\n"; 
      }else if(strcmp(argv[1],"5")==0){
        cout << i << " Sensor X1: " << nextsensor[0][0] << ", " << nextsensor[0][1] << endl;
        cout << i << " Motor Y1: " << nextmotor[0][0] << ", " << nextmotor[0][1] << endl;
        cout << i << " Sensor X2: " << nextsensor[1][0] << ", " << nextsensor[1][1] << endl;
        cout << i << " Motor Y2: " << nextmotor[1][0] << ", " << nextmotor[1][1] << endl;
        cout << i << " Sensor X3: " << nextsensor[2][0] << ", " << nextsensor[2][1] << endl;
        cout << i << " Motor Y3: " << nextmotor[2][0] << ", " << nextmotor[2][1] <<endl;   
        cout << i << " Sensor X4: " << nextsensor[3][0] << ", " << nextsensor[3][1] <<endl;
        cout << i << " Motor Y4: " << nextmotor[3][0] << ", " << nextmotor[3][1]<< endl; 
        outputvalue << nextsensor[0][0] << "," << nextmotor[0][0] << ","; // for 5 layers purpose
        outputvalue << nextsensor[1][0] << "," << nextmotor[1][0] << ","; 
        outputvalue << nextsensor[2][0] << "," << nextmotor[2][0] << ","; 
        outputvalue << nextsensor[3][0] << "," << nextmotor[3][0] << ",\n";
      }
      // outputvalue << nextsensor[0][0] << "," << nextmotor[0][0] << ","; // for 10 layers purpose
      // outputvalue << nextsensor[1][0] << "," << nextmotor[1][0] << ","; 
      // outputvalue << nextsensor[2][0] << "," << nextmotor[2][0] << ","; 
      // outputvalue << nextsensor[3][0] << "," << nextmotor[3][0] << ","; 
      // outputvalue << nextsensor[4][0] << "," << nextmotor[4][0] << ","; 
      // outputvalue << nextsensor[5][0] << "," << nextmotor[5][0] << ","; 
      // outputvalue << nextsensor[6][0] << "," << nextmotor[6][0] << ","; 
      // outputvalue << nextsensor[7][0] << "," << nextmotor[7][0] << ","; 
      // outputvalue << nextsensor[8][0] << "," << nextmotor[8][0] << ",\n"; 
    }
  }
  delete main_controller;
  if(strcmp(argv[1],"1")==0){
    delete controller0;
  }else if(strcmp(argv[1],"2")==0){
    delete controller0;
    delete controller1;
  }else if(strcmp(argv[1],"3")==0){
    delete controller0;
    delete controller1;
    delete controller2;
  }else if(strcmp(argv[1],"5")==0){
    delete controller0;
    delete controller1;
    delete controller2;
    delete controller3;
    delete controller4;
  }
  inputvalue.close();
  outputvalue.close();
  return 0;
}