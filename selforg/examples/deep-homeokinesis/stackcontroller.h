#ifndef __STACKCONTROLLER_H
#define __STACKCONTROLLER_H

#include <iostream>
#include <selforg/controller/invertmotorcontroller.h>
#include <selforg/controller/teachable.h>
#include <selforg/matrix/matrix.h>
#include <vector>

using namespace std;

typedef struct StackControllerConf{
  int buffersize; // size of the time-buffer for x,y,eta
  int nlayers; // size of the deep networks
  bool someInternalParams; // if true, only some internal parameters are exported, all otherwise
  //int numberContext; // number of context sensors(considered at the end of the sensor vector, if not 0 then S will only get them as input)
  double cInit; // size of the C matrix (controller) to initialised with
  double cNonDiag; // size of the non diagonal elements in respect to the diagonal (cInit) ones
  double cNonDiagAbs; // value of the nondiagonal elements
  //bool useS; // decide whether to use the S matrix in addition to A matrix (uses sensors as an extended model)
  //bool useSD; // decide whether to use the SD matrix (S derivative) in addition to the A matrix (uses first and second derivative as an extended model)
} StackControllerConf; 

class StackController : public InvertMotorController, public Teachable{
public: 
   // Default Constructor
  StackController(const StackControllerConf& conf=getDefaultConf());

  virtual void init(int sensornumber, int motornumber, int layernumber, RandGen* randGen = 0);

  // Destructor
  virtual ~StackController();

  static StackControllerConf getDefaultConf(){
    StackControllerConf c;
    c.buffersize = 50;
    c.nlayers = 2;
    c.someInternalParams = true;
    //c.numberContext =0;
    c.cInit =1.0;
    c.cNonDiag =0;
    c.cNonDiagAbs =0.0;
    //c.useS = false;
    //c.useSD = false;
    return c;
  }
  
  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calculates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors, motor* , int number_motors);

  //===== STOREABLE ======// 
  // stores the controller values to a given file.
  virtual bool store(FILE* f) const;

  // loads the controller values from a given file.
  virtual bool restore(FILE* f);
  /*
  //===== INSPECTABLE =====//
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;
  */
  //===== TEACHABLE =========//
  // The given motor teaching signal is used for this timestep.
  // It is used as a feed forward teaching signal for the controller.
  // Please note, that the teaching signal has to be given each timestep
  // for a continuous teaching process.
  
  //virtual void setMotorTeachingSignal(const motor* teaching, int len);

  // The given sensor teaching signal (distal learning) is used for this timestep.
  // First the belonging motor teachung signal is calculated by the inverse model.
  // See setMotorTeachingSignal
  //virtual void setSensorTeachingSignal(const sensor* teaching, int len);

  //===== New TEACHING interface ======//
  // The given motor teaching signal is used for this timestep.
  //  It is used as a feed forward teaching signal for the controller.
  //  Please note, that the teaching signal has to be given each timestep
  //  for a continuous teaching process.
  //  @param teaching: matrix with dimensions (motornumber,1)
  virtual void setMotorTeaching(const matrix::Matrix& teaching);

  // The given sensor teaching signal (distal learning) is used for this timestep.
  //  The belonging motor teachung signal is calculated by the inverse model.
  //  See setMotorTeaching
  // @param teaching: matrix with dimensions (motorsensors,1)
  virtual void setSensorTeaching(const matrix::Matrix& teaching);
  /// returns the last motor values (useful for cross motor coupling)
  virtual matrix::Matrix getLastMotorValues();
  /// returns the last sensor values (useful for cross sensor coupling)
  virtual matrix::Matrix getLastSensorValues();

protected:
  StackControllerConf conf;
  unsigned short number_sensors;
  unsigned short number_motors;

  NoiseGenerator* BNoiseGen; ///< Noisegenerator for noisy bias
  NoiseGenerator* YNoiseGen; ///< Noisegenerator for noisy motor output
  
  double E_val; ///< value of Error function
  double xsi_norm; ///< norm of matrix
  double xsi_norm_avg; ///< average norm of xsi (used to define whether Modell learns)
  double pain;         ///< if the modelling error (xsi) is too high we have a pain signal
  bool useTeaching; ///< flag whether there is an actual teachning signal or not
  int t_rand; ///< initial random time to avoid syncronous management of all controllers

  /*
  // factor to teach for continuity: subsequent motor commands should not differ too much
  double continuity;
  double modelCompliant; ///< learning factor for model (or sensor) compliant learning
  int managementInterval; ///< interval between subsequent management function calls
  paramval inhibition; ///< inhibition strength for sparce kwta strategy (is scaled with epsC)
  paramval kwta;       ///< (int) number of synapses that get strengthend
  paramval limitRF;    ///< (int) receptive field of motor neurons (number of offcenter sensors) if null then no limitation. Mutual exclusive with inhibition
  paramval dampS;     ///< damping of S matrix
  paramval dampC;     ///< damping of C matrix
  paramval activeExplore; ///< decides whether and how strong the backpropagated error is used as a control signal
  paramval cfactor;
  paramval cnondiagabs;
  paramval cdiagabs;

  paramval noiseY; ///< noise strength for y
  */
   /// puts the sensors in the ringbuffer, generate controller values and put them in the
  //  ringbuffer as well
  virtual void fillBuffersAndControl(const sensor* x_, int number_sensors,
                             motor* y_, int number_motors);

  /// calculates the first shift into the motor space useing delayed motor values.
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void calcEtaAndBufferIt(int delay);

  /// calculates xsi for the current time step using the delayed y values
  //  and x delayed by one
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void calcXsi(int delay);

  /// learn H,C with motors y and corresponding sensors x
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void learnController(int delay);

  /// calculates the Update for C and H
  // @param delay timesteps to delay the y-values.  (usually 0)
  //  Please note that the delayed values are NOT used for the error calculation
  //  (this is done in calcXsi())
  virtual void calcCandHUpdates(matrix::Matrix& C_update, matrix::Matrix& H_update, int delay);

  /// updates the matrix C and H
  virtual void updateCandH(const matrix::Matrix& C_update, const matrix::Matrix& H_update, double squashSize);

  /// learn A, (and S) using motors y and corresponding sensors x
  //  @param delay 0 for no delay and n>0 for n timesteps delay in the time loop
  virtual void learnModel(int delay);

  /// calculates the predicted sensor values
  virtual matrix::Matrix model(const matrix::Matrix* x_buffer, int delay, const matrix::Matrix& y);

  /// handles inhibition damping etc.
  virtual void management();

  /// returns controller output for given sensor values
  virtual matrix::Matrix calculateControllerValues(const matrix::Matrix& x_smooth);

  // Calculates first and second derivative and returns both in on matrix (above).
  //  We use simple discrete approximations:
  //  \f[ f'(x) = (f(x) - f(x-1)) / 2 \f]
  //  \f[ f''(x) = f(x) - 2f(x-1) + f(x-2) \f]
  //  where we have to go into the past because we do not have f(x+1). The scaling can be neglegted.
  
  matrix::Matrix calcDerivatives(const matrix::Matrix* buffer, int delay);
*/
};
#endif