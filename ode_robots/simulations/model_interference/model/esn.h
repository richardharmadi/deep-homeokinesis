/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                   Echo State Network Header File                          *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/
#ifndef __ESN_H
#define __ESN_H


#include <stdio.h>
#include <cmath>
#include "homeokineticmodel.h"
#include <selforg/matrix.h>


struct ESNConf {
  int numNeurons;            ///< number of neurons in the reservoir
  double inputStrength;      ///< strength of input to reservoir connections
  double inputRatio;         ///< ratio of input connections w.r.t full connectivity
  double fbStrength;        // Feedback strenght from output (or teaching force) to reservoir
  double fbRatio;           // Ratio of output (or teaching force) to reservoir
  double connectionRatio;    ///< ratio of internal connections w.r.t full connectivity
  double spectralRadius;     ///< largest eigenvalue of internal weights
  double learningRate;
  /// switch on to get the internal weights and activations inspectabe
  bool    inspectInternals;
  bool    useExtendedModel;
};


/**
 * Echo State Network modifieded for easy us for Homeokinesis (soxmm.h) model
 * x^(tilde) = M(x, y) = M(x, K(x) = g(Wos * s + Wox * x + Woy * y)
 * s: internal state (ESNstate), no feedback, f as identity
 * Internal state update: s(t+1) = W * s(t) + Wsx * x(t+1) + Wsy * y(t+1)
 */
class ESN : public HomeokineticModel {
public:

  /**
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  ESN(const ESNConf& conf = getDefaultConf());
  virtual ~ESN() {};

  static ESNConf getDefaultConf() {
    ESNConf c;
    c.numNeurons = 100;
    c.inputStrength = 1.;
    c.inputRatio =1.;
    c.fbStrength = 1.;
    c.fbRatio = 1.;
    c.connectionRatio = 0.1;
    c.spectralRadius  = 0.9;
    c.inspectInternals = false;
    c.learningRate=0.01;
    c.useExtendedModel = true;
    return c;
  }

  /** initialisation of the network with the given number of input and output units
      @param y_inputDim length of input vector (nMotors)
      @param x_inputDim length of input vector (nSensors)
      @param outputDim length of output vector
      @param unit_map if 0 the parametes are choosen randomly.
             Otherwise the model is initialised to represent a unit_map
             with the given response strength.
      @param randGen pointer to random generator, if 0 an new one is used
   */
  virtual void init(unsigned int y_inputDim, unsigned int x_inputDim, unsigned  int outputDim,
                    double unit_map, RandGen* randGen);
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  /** passive processing of the input
     (this function is not constant since a recurrent network
     for example might change internal states
  */
  virtual const matrix::Matrix process (const matrix::Matrix& x_input, const matrix::Matrix& y_input);
  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /*virtual const matrix::Matrix process (const matrix::Matrix& input,const matrix::Matrix& ts);*/
  /* performs learning and returns the network output before learning.
     Neural networks process the input before. (no need to call process before)
     \param learnRateFactor can be given to modify eps for this learning step.
  */
  virtual const matrix::Matrix learn (const matrix::Matrix& x_input,
                                      const matrix::Matrix& y_input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1.);
  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1.);

  /* calculates the partial derivative of the of the output with repect to the input (Jacobi matrix).

      \f[J_{ij} = \frac{\partial output_i}{\partial input_j}\f]

      The input is ignored, the network must  be processed or learned before!
   */
  virtual const matrix::Matrix response(const matrix::Matrix& _input) const;


  /* calculates the partial derivative of the of the output with repect to the control
   * parameter.

      \f[J_{ij} = \frac{\partial output_i}{\partial parameter_j}\f]

      The input is ignored, the network must  be processed or learned before!
   */
  virtual const matrix::Matrix response_controller() const;
  virtual const matrix::Matrix response_controller2() const;


  /* calculates the input shift v to given output shift xsi via pseudo inversion.

      \f[o+\xi = \pi(i+v)\f]

      The input is ignored, the network must  be processed or learned before!
   */
  virtual const matrix::Matrix inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const;

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping);

  /// returns the number of input neurons
  virtual unsigned int getInputDim() const;
  /// returns the number of output neurons
  virtual unsigned int getOutputDim() const;

  virtual bool store(FILE*) const;

  virtual bool restore(FILE*);

  virtual bool getInitialized() {
    return initialized;
  }

  static double tanh_prime(double z)
  {
    double k=tanh(z);
    return 1.0 - k*k;
  };

  virtual bool getIsInitialized();

protected:

  ESNConf conf;
  /*matrix::Matrix inputWeights; //W_in_res all-to-all random-value no-learn*/
  /* For easiness of use with HK controllers input is divided into
   * (x, y) = (sensors, motors) */ 
  matrix::Matrix Wsx; //W_in_res (input of x to reservoir) all-to-all random-value no-learn
  matrix::Matrix Wsy; //W_in_res (input of y to reservoir) all-to-all random-value no-learn
  /*matrix::Matrix fbWeights; //W_out_res all-to-all random-value no-learn*/
  matrix::Matrix Woy; // W_in_out (input of y to output) all-to-all learn
  matrix::Matrix Woy_native; // Matrix for damping
  matrix::Matrix Wox; // W_in_out (input of x to output) all-to-all learn
  matrix::Matrix Wos; // W_res_out all-to-all 0-value learn
  /*matrix::Matrix outputDirectWeights; // W_in_out all-to-all learn*/
  matrix::Matrix ESNWeights; // W_res_res random_number random-value no-lear normilised-by-spectral_radius

  matrix::Matrix ESNState; // State of reservoir after applying k = tanh
  matrix::Matrix ESNActivations; // State of reservoir before applying k

  paramval causeaware;
  paramval damping;
  double error;
  bool initialized;

  //
};

#endif

