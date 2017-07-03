#ifndef __STACKINVERTMOTORNSTEP_H
#define __STACKINVERTMOTORNSTEP_H

#include "invertmotornstep.h"
#include "nstepwrapper."
#include <vector>

/**
 * class for robot controller that uses the georg's invertmotornstep network
 * (deep networks)
 */
class StackInvertMotorNStep : public InvertMotorController, public Teachable{

public:
  vector<NStepWrapper> controllers; // vector of controller that will be stacked
  int nlayers;

  StackInvertMotorNStep(int nlayers);

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~StackInvertMotorNStep();

  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /**** INSPECTABLE ****/
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;


  //**** New Stack functions ********//
  matrix::Matrix& getC(int layernumber);
  matrix::Matrix& getA(inn layernumber);
  virtual matrix::Matrix getPredictionFromLayer(int layernumber);

protected:
  unsigned short number_sensors;
  unsigned short number_motors;
};

#endif
