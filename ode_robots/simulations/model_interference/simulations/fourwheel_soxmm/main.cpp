//// Simulation
#include <ode_robots/simulation.h>
// Noise generator
#include <selforg/noisegenerator.h>
// Agent: bind robot, controller and wiring together
#include <ode_robots/odeagent.h>
// Robot's controller
#include "../../controller/soxmm.h"
#include <selforg/controller/invertmotornstep.h>
#include <selforg/controller/stackinvertmotornstep.h>

// Robot's wiring
#include <selforg/one2onewiring.h>
// The robot
#include <ode_robots/fourwheeled.h> 
// used arena
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>

#include <gsl/gsl_histogram.h>
#include "../../model/esn.h"
#include <ode_robots/terrainground.h>

#include <selforg/logeable.h>

using namespace lpzrobots;
int ESN_N;
bool toLog;
double zsize;
int which_model;
double epsA;
double epsC;
double spectral_radius;
int terrain;
string logFileName;
bool useExtendedModel;
double stuckness;
double sigma_sqr_LWR;

class ThisSim : public Simulation
{
  public:
    int bin_x, bin_y, coverage, displacement;
    OdeRobot* robot;
    SoxMM* controller;
    StackInvertMotorNStep* controller_inv;
    InvertMotorNStep* controller0;
    InvertMotorNStep* controller1;
    Logeable* log;
    double error;
    int cover[10][10];
    bool upside_down;
    Pos position;

    ThisSim() { }

    ~ThisSim() {
      stuckness = 100. * stuckness / globalData.sim_step;
      cout << "stuck percentage: " << stuckness << endl;
      if (toLog) {
        switch(which_model) {
          case 1: {
            log = new Logeable(logFileName, "coverage\terror\tzsize\tepsa\tepsc\tdisplacement\tupdise_down\ttime_steps\textended_model\tmodel\tstuck_percentage");
            string ss;
            ss = to_string(coverage)  + "\t" + to_string(error) + "\t" + to_string(zsize) 
                + "\t" + to_string(epsA) + "\t" + to_string(epsC) + "\t" 
                + to_string(displacement) + "\t" + to_string(upside_down) 
                + "\t" + to_string(globalData.sim_step) + "\t" + to_string(useExtendedModel)
                + "\t" + to_string(which_model) + "\t" + to_string(stuckness);
            log->toLog(ss);
            break;
            }
          case 2: {// ESN
            log = new Logeable(logFileName, "ens_n\tcoverage\terror\tzsize\tepsa\tepsc\tspectral_radius\tdisplacement\tupdise_down\ttime_steps\textended_model\tmodel\tstuck_percentage");
            string ss;
            ss = to_string(ESN_N) + "\t" + to_string(coverage)  + "\t" + to_string(error)
                  + "\t" + to_string(zsize) + "\t" + to_string(epsA) 
                  + "\t" + to_string(epsC) + "\t" + to_string(spectral_radius)
                  + "\t" + to_string(displacement) + "\t" + to_string(upside_down) 
                  + "\t" + to_string(globalData.sim_step) + "\t" + to_string(useExtendedModel)
                  + "\t" + to_string(which_model) + "\t" + to_string(stuckness);
            log->toLog(ss); 
            break;
            }
          case 3: { // LINEAR REGRESSION
            log = new Logeable(logFileName, "coverage\terror\tzsize\tepsa\tepsc\tdisplacement\tupdise_down\ttime_steps\textended_model\tmodel\tstuck_percentage");
            string ss;
            ss = to_string(coverage)  + "\t" + to_string(error) + "\t" + to_string(zsize) 
                + "\t" + to_string(epsA) + "\t" + to_string(epsC) 
                + "\t" + to_string(displacement)+ "\t" + to_string(upside_down) 
                + "\t" + to_string(globalData.sim_step) + "\t" + to_string(useExtendedModel)
                + "\t" + to_string(which_model) + "\t" + to_string(stuckness);
            log->toLog(ss);
            break;
            }
          case 4: { // LWR
            log = new Logeable(logFileName, "coverage\terror\tzsize\tsigma_sqr\tepsc\tdisplacement\tupdise_down\ttime_steps\textended_model\tmodel\tstuck_percentage");
            string ss;
            ss = to_string(coverage)  + "\t" + to_string(error) + "\t" + to_string(zsize) 
                + "\t" + to_string(sigma_sqr_LWR) + "\t" + to_string(epsC) 
                + "\t" + to_string(displacement)+ "\t" + to_string(upside_down) 
                + "\t" + to_string(globalData.sim_step) + "\t" + to_string(useExtendedModel)
                + "\t" + to_string(which_model) + "\t" + to_string(stuckness);
            log->toLog(ss);
            break;
            }
        }
      }
    }
     
    /// start() is called at the start and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
      // Initial position and orientation of the camera (use 'p' in graphical window to find out)
      setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
      // Some simulation parameters can be set here
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);

      /** New robot instance */
      // Get the default configuration of the robot
      FourWheeledConf robotConf = FourWheeled::getDefaultConf();
      robotConf.useBumper = false;
      robotConf.useButton = false;
      robotConf.useBigBox = true;
      robotConf.force     = 10;
      // Instantiating the robot
      robot = new FourWheeled(odeHandle, osgHandle, robotConf, "Four wheel robot");
      // Placing the robot in the scene
      ((OdeRobot*)robot)->place(Pos(.0, .0, .1));
      // Instantiating the model and controller

      SoxMMConf soxConf = SoxMM::getDefaultConf();
      soxConf.initFeedbackStrength = 1.2;
      soxConf.useExtendedModel = useExtendedModel;
      switch(which_model) {
        case 1: // Linear
          controller = new SoxMM(soxConf, LINEAR);
          controller->setParam("epsA", epsA); // 0.05
	  controller_inv = new StackInvertMotorNStep(50,10);
	  controller0 = new InvertMotorNStep();
	  controller1 = new InvertMotorNStep();
          controller_inv->addLayer(controller0);
          controller_inv->addLayer(controller1);
	  controller0->setParam("epsA",0.05);
	  controller1->setParam("epsA",0.05);
	  controller0->setParam("epsC",0.2);
	  controller1->setParam("epsC",0.2);
          break;
        case 3: // Linear Regression
          controller = new SoxMM(soxConf, LINEAR_REGRESSION);
          controller->setParam("epsA", epsA); // 0.05
          break;
        case 4: // LW Regression
          controller = new SoxMM(soxConf, LWR);
          //controller->setParam("epsA", epsA); // 0.05 -- there is no learning rate for the model --
          controller->setParam("sigma_sqr_LWR", sigma_sqr_LWR); // 0.2
          break;
        case 2: // ESN
          ESNConf modelConf = ESN::getDefaultConf();
          modelConf.learningRate = epsA;
          modelConf.inspectInternals = false;
          modelConf.numNeurons = ESN_N;
          modelConf.spectralRadius = spectral_radius;
          modelConf.useExtendedModel = useExtendedModel;
          modelConf.inspectInternals = false; 
          ESN* model = new ESN(modelConf);
          controller = new SoxMM(model, ESN_MODEL, soxConf);
          break;
      }
      controller->setParam("epsC", epsC); // 0.3
      controller->setParam("Logarithmic", 0);

      // Create the wiring with color noise
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));
      // Create Agent
      OdeAgent* agent = new OdeAgent(global);
      // Agent initialisation
      agent->init(controller_inv, robot, wiring);
      // Adding the agent to the agents list
      global.agents.push_back(agent);
      global.configs.push_back(agent);

      // Playground
      if (zsize >= .0) {
        double widthground = 20.;
        double heightground = 1.5;
        Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground));
        playground->setColor(Color(1., 1., 1., .99));
        playground->setPosition(osg::Vec3(0, 0, .0));
        if (terrain == 0) {
          Substance substance;
          substance.toRubber(5);
          playground->setGroundSubstance(substance);
          global.obstacles.push_back(playground);
          double xboxes = 18;//19.0;
          double yboxes = 17;
          double boxdis = 3.;//.45;//1.6;
          for (double j = .0; j < xboxes; j++)
            for(double i = .0; i < yboxes; i++) {
              double xsize = 1.;//1.0;
              double ysize = 1.;//.25;
              PassiveBox* b =
                new PassiveBox(odeHandle,
                    osgHandle, osg::Vec3(xsize,ysize,zsize),0.0);
              b->setPosition(Pos( + boxdis*(i-(xboxes-1)/2.0), + boxdis*(j-(yboxes-1)/2.0), 0.01));
              global.obstacles.push_back(b);
            }
        }
        if (terrain == 1) {
          TerrainGround* terrainground =
            new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0f,1.0f)),
                "../../utils/terrain.ppm", "../../utils/maze256c.ppm",
                20, 20, zsize, OSGHeightField::Red);
          terrainground->setPose(osg::Matrix::translate(.0, .0, .11));
          global.obstacles.push_back(terrainground);
        }
      }

      bin_x = 0;
      bin_y = 0;
      coverage = 0;
      displacement = 0;
      upside_down = false;
      stuckness = .0;
      for (int i = 0; i < 10; i++)
        for (int j = 0; j < 10; j++)
          cover[i][j] = 0;

}

    /* Functions not used in this tutorial but typically useful */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
      if (toLog) {
        Position robot_position = robot->getPosition();
        const int playground = 20;
        const int bins = 10;
        double rx = robot_position.x + (playground / 2);
        double _bin_x = floor(rx / (playground / bins));
        double ry = robot_position.y + (playground /2);
        double _bin_y = floor(ry / (playground / bins));

        if (((fmod(rx, playground / bins) < 0.2) && (bin_x != _bin_x))  ||
            ((fmod(ry, playground / bins) < 0.2) && (bin_y != _bin_y))) {
          bin_x = _bin_x;
          bin_y = _bin_y;
          if (cover[bin_x][bin_y] == 0)
            coverage++;
          cover[bin_x][bin_y]++;
          displacement++;
        }

      }

      // this should be called at the end of the simulation but controller
      // already dead at ~ThisSim()
      error = controller->getXiAverage();
      
      Pose pp = robot->getMainPrimitive()->getPose();
      if (pp(0,2) < -.8) {
        simulation_time_reached = true;
        upside_down = true;
      }

      Pos po = robot->getMainPrimitive()->getPosition();
      if ((abs(position.x() - po.x()) < 0.000001) &&
          (abs(position.y() - po.y()) < 0.000001) &&
          (abs(position.z() - po.z()) < 0.000001)) {
        stuckness++;
      }
      position = po;
    }

    virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
      return false;
    }

    virtual void bindingDescription(osg::ApplicationUsage & au) const {
    }
};

int main (int argc, char **argv)
{
  // New simulation
  ThisSim sim;
  // set Title of simulation
  sim.setTitle("SOXMM+ESN");
  sim.setCaption("Smith Polydoros Herrmann");
  int index;

  index = Base::contains(argv, argc, "-esn_n");
  if(index) {
    if(argc > index)
      ESN_N = atoi(argv[index]);
  } else {
    ESN_N = 100;
  }
  
  toLog = false;
  logFileName = "";
  index = Base::contains(argv, argc, "-log");
  if (index) {
    if (argc > index) {
       logFileName = argv[index];
       toLog = true;
     }
  }

  // controller learning rate
  epsC = .005; // 0.3;
  index = Base::contains(argv, argc, "-epsc");
  if(index) 
    if(argc > index)
      epsC = atof(argv[index]);

  // model learning rate
  epsA = .005; // 0.05;
  index = Base::contains(argv, argc, "-epsa");
  if(index) 
    if(argc > index)
      epsA = atof(argv[index]);

  // sigma square LWR
  sigma_sqr_LWR = .2;
  index = Base::contains(argv, argc, "-sigma_sqr_LWR");
  if(index) 
    if(argc > index)
      sigma_sqr_LWR = atof(argv[index]);

  // Negative values cancel playground
  zsize = -1.;
  index = Base::contains(argv, argc, "-playground");
  if(index) 
    if(argc > index)
      zsize = atof(argv[index]);

  terrain = 0; // boxes
  index = Base::contains(argv, argc, "-terrain");
  if(index) 
    if(argc > index)
      terrain = atoi(argv[index]);


  which_model = 1; // ESN as default
  index = Base::contains(argv, argc, "-model");
  if(index) 
    if(argc > index)
      which_model = atoi(argv[index]);

  spectral_radius = .9; 
  index = Base::contains(argv, argc, "-spectralradius");
  if(index) 
    if(argc > index)
      spectral_radius = atof(argv[index]);

  useExtendedModel = true;
  index = Base::contains(argv, argc, "-noextendedmodel");
  if(index) 
    useExtendedModel = false;

  // Simulation begins
  return sim.run(argc, argv) ? 0 : 1;
}
