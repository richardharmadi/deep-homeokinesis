#ifndef __STACKINVERTMOTORNSTEP_H
#define __STACKINVERTMOTORNSTEP_H

#include "invertmotornstep.h"
#include <vector>

using namespace std;
/**
 * class for robot controller that uses the georg's invertmotornstep network
 * (deep networks)
 */
class StackInvertMotorNStep : public InvertMotorController{

public:
  vector<InvertMotorNStep> controllers; // vector of controller that will be stacked

  StackInvertMotorNStep(int buffersize, int nlayers);

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual void addLayer(InvertMotorNStep cont);
  
  virtual ~StackInvertMotorNStep();

  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; };
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; };

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /**** INSPECTABLE ****/
  //virtual std::list<ILayer> getStructuralLayers() const;
  //virtual std::list<IConnection> getStructuralConnections() const;

  /**** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  //**** New Stack functions ********//
  //matrix::Matrix& getC(int layernumber);
  //matrix::Matrix& getA(int layernumber);
  virtual sensor* getPredInputFromLayer(int layernumber);
  virtual motor* getInvOutputFromLayer(int layernumber);
  virtual sensor* getInvInputFromLayer(int layernumber);
  virtual int getNLayer() const { return actual_nlayer; };

protected:
  unsigned short number_sensors;
  unsigned short number_motors;

private:
  int actual_nlayer = 0;
  int buffersize ;
  vector<sensor*>pred_x;
  vector<motor*>inv_y;
  vector<sensor*>inv_x; // reconstructed input starting from first layer
  vector<motor*>ynext; // output starting from the second layer
};

#endif
