/** \file RobovieWalkerEnv.hh
   Definisce l'ambiente per la camminata a frame del RobovieX
    \author agatto
*/

#ifndef _ROBOVIEXWALKERENV_H_
#define _ROBOVIEXWALKERENV_H_

#include <set>
#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <sensor_msgs/JointState.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "gazebo_msgs/GetModelState.h"




class RobovieWalkerEnv: public Environment {
public:

  /** Creates il dominio per lo sviluppo della camminata del RobovieX.
   * INPUT : rand - generatore numeri aleatori
  */
  RobovieWalkerEnv(Random &rand);

  /** Creates il dominio per lo sviluppo della camminata del RobovieX.
    * INPUT : rand - generatore numeri aleatori
    *         isReale - booleano per selezionare tra robot reale e simulato
  */
  RobovieWalkerEnv(Random &rand, bool isReale);

  virtual ~RobovieWalkerEnv();

  //Vettore di output
  virtual const std::vector<float> &sensation() const;
  virtual float apply(int action);

  void setLegData(double legA,double legB){
  	  m_legALength=legA;
  	  m_legBLength=legB;
  	  m_legLength=legA+legB;
  	};

   //Inizializzare così questi due parametri
   //SimpleWalk_2(){m_state=0; m_paramNumber=9;};


	//void executeAction(Genome g,long time,HumanoidJoints &hj);

	//setta il range di ogni parametro
	//void getMask(Genome &g);

	//void init(Genome g,HumanoidJoints &hj);

	void init();

//--------------------------------------------------------------------------------
  /** Calculate the new state and reward for the given force */
  float transition(float force);

  virtual bool terminal() const;
  virtual void reset();

  virtual int getNumActions();
  virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
  virtual void getMinMaxReward(float* minR, float* maxR);

  /** Set the state vector (for debug purposes) */
  void setSensation(std::vector<float> newS);

  virtual std::vector<experience> getSeedings();

  /** Get an experience for the given state-action */
  experience getExp(float s0, float s1, float s2, float s3,float s4,float s5,float s6,float s7,float s8, int a);
  //Posizione di ritorno
       geometry_msgs::Pose pose_ritorno;

 //Mettiamo i publisher globali
 ros::NodeHandle nh;
 ros::Publisher pub;

protected:
  enum car_action_t {LEFT, RIGHT};

  //Variabili
  float xfin, yfin, zfin, xprec,yprec;
  int steps;
  bool leg_l, leg_r;
  bool walk_finished;
  long nframes;
  double fstable;


     //Nomi per definire le VARIABILI della camminata
     static const int TIME=0;
     static const int HIP_R=1;
     static const int THIGH=2;
     static const int KNEE=3;
     static const int EX_KNEE=4;
     static const int EX_ANKLE=5;
     static const int BODY = 6;
     static const int ANKLE_R = 7;
     static const int SHOULDER = 8;

     

     //Numero di parametri
     int m_paramNumber;

     //Stato iniziale
     int m_state;

     //Definisce il tipo di simulazione
     bool tipoSimulazione;

     //Vettore che rappresenta i frame della camminata
     sensor_msgs::JointState frame[7];
     sensor_msgs::JointState hj;


     //Lunghezza delle gambe del robot
     double m_legALength;
     double m_legBLength;
     double m_legLength;

     ros::NodeHandle* nh_;

     bool m_rightLegUp; //it's true if the robot lifts up the right leg
      bool m_leftLegUp;
      int m_stepNumber; // number of step executed
      bool m_actionFinished;
      bool m_forceToFinish;


private:

  std::vector<float> s;
  
  long timestamp, time0;
  long age;			// in milliseconds
  long startTime;
   
  //parametri
     float &time_p;
     float &hip_r_p;
     float &thigh_p;
     float &knee_p;
     float &ex_knee_p;
     float &ex_ankle_p;
     float &body_p;
     float &ankle_r_p;
     float &shoulder_p;

  //------------------------------------------------------------------------------------
  float GRAVITY;
  float MASSCART;
  float MASSPOLE;
  float TOTAL_MASS;
  float LENGTH;

  float POLEMASS_LENGTH;
  float FORCE_MAG;
  float TAU;

  float FOURTHIRDS;
  float DEG_T_RAD;
  float RAD_T_DEG;

  const bool noisy;
  Random &rng;

  /*float &cartPos;
  float &cartVel;
  float &poleAngle;
  float &poleVel;*/

  float reward();

  int recuperaPosJoint(sensor_msgs::JointState giunto,const std::string& nome);

  void sendPos(sensor_msgs::JointState msg);

  void executeStep ();

  bool getObjectPose (std::string model_name, geometry_msgs::Pose &pose);

  bool resetRobotGazebo (std::string model_name);

  bool resetRobotGazeboMorbido (std::string model_name);

  void resetWalk();

  void executeAction(long time /*,HumanoidJoints &hj*/);

  double evaluateFitness(geometry_msgs::Pose &pose);

  //------------------------------------------------------------------------------------


};

#endif
