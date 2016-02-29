/** \file RobovieWalkerEnv.cpp
 Definisce l'ambiente per la camminata a frame del RobovieX
 \author agatto
 */

#include <rl_env/RobovieWalkerEnv.hh>
#include <vector>

//Costruttore
RobovieWalkerEnv::RobovieWalkerEnv(Random &rand) :
			//Inizializzazione ambiente
			//Inizializzazione vettore parametri s - 9 parametri da ottimizzare con RL
			/*
			 float &time_p;
			 float &hip_r_p;
			 float &thigh_p;
			 float &knee_p;
			 float &ex_knee_p;
			 float &ex_ankle_p;
			 float &body_p;
			 float &ankle_r_p;
			 float &shoulder_p;
			 */
			noisy(false), rng(rand), s(9), time_p(s[0]), hip_r_p(s[1]),
			thigh_p(s[2]), knee_p(s[3]), ex_knee_p(s[4]), ex_ankle_p(s[5]),
			body_p(s[6]), ankle_r_p(s[7]), shoulder_p(s[8]) {
	pub = nh.advertise<sensor_msgs::JointState> ("joint_states", 1);
	//cout << "\n DENTRO \n";
	//Non server il reset
	reset();
	//cout << *this << endl;
}

//Secondo costruttore
RobovieWalkerEnv::RobovieWalkerEnv(Random &rand, bool isReale) :
	noisy(false), rng(rand), s(9), time_p(s[0]), hip_r_p(s[1]), thigh_p(s[2]),
			knee_p(s[3]), ex_knee_p(s[4]), ex_ankle_p(s[5]), body_p(s[6]),
			ankle_r_p(s[7]), shoulder_p(s[8]) {
	pub = nh.advertise<sensor_msgs::JointState> ("joint_states", 1);
	//Sto pubblicando sul jointState, quindi dipende se ho attivo il nodo del reale e quello di gazebo
	tipoSimulazione = isReale;
	//cout << "\n DENTRO isReale \n";
	//Non server il reset
	reset();
}

//Distruttore
RobovieWalkerEnv::~RobovieWalkerEnv() {
}

//Recupera la experience
experience RobovieWalkerEnv::getExp(float s0, float s1, float s2, float s3,
		float s4, float s5, float s6, float s7, float s8, int a) {
	//cout << "\n DENTRO ::getExp \n";
	experience e;

	e.s.resize(9, 0.0);
	e.next.resize(9, 0.0);

	//cout  << "s0=" << s0 << "\n DENTRO ::getExp despues resize s \n" ;

	time_p = s0;
	//cout << "\n DENTRO ::getExp despues valorizar s \n";
	hip_r_p = s1;
	thigh_p = s2;
	knee_p = s3;
	ex_knee_p = s4;
	ex_ankle_p = s5;
	body_p = s6;
	ankle_r_p = s7;
	shoulder_p = s8;

	cout << "\n DENTRO ::getExp VALORES:"<<s0<<","<<s1<<","<<s2<<","<<s3<<","<<s4<<","<<s5<<","<<s6<<","<<s7<<","<<s8<<"\n";


	e.act = a;
	e.s = sensation();
	e.reward = apply(e.act);

	e.terminal = terminal();
	e.next = sensation();

	return e;
}

//Ritorna il vettore sensation
const std::vector<float> &RobovieWalkerEnv::sensation() const {//cout << "\n DENTRO sensation \n";
	//cout << "At state " << s[0] << ", " << s[1] << endl;
	return s;
}

//Ritorna il numero di actions dell'ambiente
int RobovieWalkerEnv::getNumActions() {//cout << "\n DENTRO getNumActions \n";
	return 1;
}

//Ritorna il vettore s
void RobovieWalkerEnv::setSensation(std::vector<float> newS) {
	cout << "\n DENTRO setSensation \n";
	if (s.size() != newS.size()) {
		cerr << "Error in sensation sizes" << endl;
	}

	for (unsigned i = 0; i < newS.size(); i++) {
		s[i] = newS[i];
	}
}

void RobovieWalkerEnv::getMinMaxFeatures(std::vector<float> *minFeat,
		std::vector<float> *maxFeat) {
	//cout << "\n DENTRO getMinMaxFeatures \n";
	// set the parameters range -- DAL getMask ORIGINALE
	/*
	 g.setMask(100,3000,TIME);
	 g.setMask(0,M_PI/4,BODY);
	 g.setMask(0,M_PI/10,HIP_R);
	 g.setMask(0,M_PI/10,ANKLE_R);
	 g.setMask(0,M_PI/3,THIGH);
	 g.setMask(0,M_PI/4,KNEE);
	 g.setMask(0,M_PI/6,EX_KNEE);
	 g.setMask(0,M_PI/6,EX_ANKLE);
	 g.setMask(0,M_PI/4,SHOULDER);
	 */

	minFeat->resize(s.size(), 0.0);
	maxFeat->resize(s.size(), 1.0);

	(*minFeat)[TIME] = 100;
	(*maxFeat)[TIME] = 3000;

	(*minFeat)[BODY] = 0;
	(*maxFeat)[BODY] = M_PI / 4;

	(*minFeat)[HIP_R] = 0;
	(*maxFeat)[HIP_R] = M_PI / 10;

	(*minFeat)[ANKLE_R] = 0;
	(*maxFeat)[ANKLE_R] = M_PI / 10;

	(*minFeat)[THIGH] = 0;
	(*maxFeat)[THIGH] = M_PI / 3;

	(*minFeat)[KNEE] = 0;
	(*maxFeat)[KNEE] = M_PI / 4;

	(*minFeat)[EX_KNEE] = 0;
	(*maxFeat)[EX_KNEE] = M_PI / 6;

	(*minFeat)[EX_ANKLE] = 0;
	(*maxFeat)[EX_ANKLE] = M_PI / 6;

	(*minFeat)[SHOULDER] = 0;
	(*maxFeat)[SHOULDER] = M_PI / 4;

}

//Recupera la posione del giunto a partire del nome
int RobovieWalkerEnv::recuperaPosJoint(sensor_msgs::JointState giunto,
		const std::string& nome) {//cout << "\n DENTRO recuperaPosJoint \n";
	for (unsigned i = 0; i < giunto.name.size(); i++) {
		if (giunto.name[i] == nome) {
			return i;
		}
	}

	return -1;
}

//inizializzazione frame camminata
void RobovieWalkerEnv::init(/*Genome g,HumanoidJoints &hj*/) {
	//cout << "\n DENTRO init \n";
	//getMask(g);

	//10.05.2015 VERIFICAR LEG DATA
	//setLegData(60,60);

	//setLegData(59,59);
	setLegData(60,60);

	double thighBase = 0;
	double kneeBase = 0;
	double ankleBase = 0;
	//double distance = m_legLength * .85;
	double distance = m_legLength * .75;
	// calculate the cynematic chain angles
	double phi_d = 3 * M_PI / 2;
	double phi_B = phi_d + acos((m_legALength * m_legALength - m_legBLength
			* m_legBLength - distance * distance) / (2 * distance
			* m_legBLength));
	phi_B -= (phi_B > 2 * M_PI) ? 2 * M_PI : 0;

	//cout << "\n ***************************************\n";
	//cout << "\n DENTRO init distance=" << distance << "\n";
	//cout << "\n DENTRO init phi_d=" <<	phi_d << "\n";
	//cout << "\n DENTRO init phi_B=" << phi_B << "\n";
	//cout << "\n ***************************************\n";

	// knee coordinates
	double x_knee = m_legBLength * cos(phi_B);
	double z_knee = m_legBLength * sin(phi_B);
	double phi_A = atan2((distance - z_knee), (-x_knee));
	double bodyp = -M_PI / 10;
	thighBase = (-M_PI / 2 + phi_A);
	kneeBase = -(phi_A - phi_B);
	//kneeBase = -(0.5);
	ankleBase = M_PI / 2 - phi_B;

	//cout << "\n ***************************************\n";
    //cout << "\n DENTRO init x_knee=" << x_knee << "\n";
	//cout << "\n DENTRO init z_knee=" << z_knee << "\n";
	//cout << "\n DENTRO init phi_A=" << phi_A << "\n";
	//cout << "\n DENTRO init thighBase=" << thighBase << "\n";
	//cout << "\n DENTRO init kneeBase=" << kneeBase << "\n";
	//cout << "\n DENTRO init ankleBase=" << ankleBase << "\n";
	//cout << "\n ***************************************\n";

	double hip = hip_r_p;//g.getMaskedGeneFloat(HIP_R);
	double ankle_r = ankle_r_p;//g.getMaskedGeneFloat(ANKLE_R);

	double thigh = thigh_p;//g.getMaskedGeneFloat(THIGH);
	double knee = knee_p;//g.getMaskedGeneFloat(KNEE);
	double ex_knee = ex_knee_p;//g.getMaskedGeneFloat(EX_KNEE);
	double ex_ankle = ex_ankle_p;//g.getMaskedGeneFloat(EX_ANKLE);
	double body = body_p;//g.getMaskedGeneFloat(BODY);
	double elbow = shoulder_p;//g.getMaskedGeneFloat(SHOULDER);

	/*
	 * COSTANTI PER I JOINT
	 head_joint_yaw = 0,					//  0
	 left_shoulder_joint_pitch,	//  1
	 right_shoulder_joint_pitch,	//  2
	 left_shoulder_joint_roll,		//  3
	 right_shoulder_joint_roll,	//  4
	 left_arm_joint_pitch,				//  5
	 right_arm_joint_pitch,			//  6
	 left_hip_joint_roll,				//  7
	 right_hip_joint_roll,				//  8
	 left_hip_joint_pitch,				//  9
	 right_hip_joint_pitch,			// 10
	 left_knee_joint_pitch,			// 11
	 right_knee_joint_pitch,			// 12
	 left_ankle_joint_pitch,			// 13
	 right_ankle_joint_pitch,		// 14
	 left_ankle_joint_roll,			// 15
	 right_ankle_joint_roll,			// 16
	 */

	//IMPORTANTE!! - ARRIVATI A QUESTO PUNTO IL ROBOT REALE/VIRTUALE DEV'ESSERE STATO INIZIALIZZATO!!
	//RICORDARSI DI LANCIARE INIZIALIZZAZIONE ROBOT!!!

	//VEDERE SE USARE INDICI CON ALIAS!!

	sensor_msgs::JointState joint_state;
	//Resetto il frame prima di usarlo
	frame[0] = joint_state;

	// start position
	//frame[0].name.resize(6);
	//frame[0].position.resize(6);
	frame[0].name.push_back("right_hip_joint_pitch");
	frame[0].name.push_back("left_hip_joint_pitch");
	frame[0].name.push_back("right_knee_joint_pitch");
	frame[0].name.push_back("left_knee_joint_pitch");

	frame[0].name.push_back("right_ankle_joint_pitch");
	frame[0].name.push_back("left_ankle_joint_pitch");


	frame[0].name.push_back("right_shoulder_joint_pitch");
	frame[0].name.push_back("left_shoulder_joint_pitch");

	frame[0].name.push_back("right_hip_joint_roll");
	frame[0].name.push_back("right_ankle_joint_roll");
	frame[0].name.push_back("left_hip_joint_roll");
	frame[0].name.push_back("left_ankle_joint_roll");

	/*
	 frame[0].name[0] ="right_hip_joint_pitch";
	 frame[0].name[1] ="left_hip_joint_pitch";
	 frame[0].name[2] ="right_knee_joint_pitch";
	 frame[0].name[3] ="right_ankle_joint_pitch";
	 frame[0].name[4] ="left_ankle_joint_pitch";
	 frame[0].name[5] ="left_knee_joint_pitch";
	 */

	frame[0].position.push_back(thighBase);
	frame[0].position.push_back(thighBase);

	frame[0].position.push_back(kneeBase);
	frame[0].position.push_back(kneeBase);

	frame[0].position.push_back(ankleBase-bodyp);
	frame[0].position.push_back(ankleBase-bodyp);

	frame[0].position.push_back(0.0);
	frame[0].position.push_back(0.0);

	frame[0].position.push_back(0.0);
	frame[0].position.push_back(0.0);
	frame[0].position.push_back(0.0);
	frame[0].position.push_back(0.0);

	/*
	for (uint i = 0; i < frame[0].position.size(); i++) {

				cout << "\n DENTRO init frame[0] - pos[i]=" << frame[0].position[i] << " [i]=" << i << "\n";
				}
    */
	/*
	 frame[0].position[0] = frame[0].position[1] = thighBase;
	 frame[0].position[2] = frame[0].position[3] = kneeBase;
	 frame[0].position[4] = frame[0].position[5] = ankleBase;
	 */
	//cout << "\n DENTRO RESIZE name[0]=" << frame[0].name[0] << "\n";
	// cout << "\n DENTRO RESIZE position[0]=" << frame[0].position[0] << "\n";

	//second position
	frame[1] = frame[0];
	//frame[1].hipL.P+=thigh;
	frame[1].position[1] += thigh;
	//frame[1].position[5] -= knee;
	//frame[1].kneeL.P -=knee;
	//Controllare il segno!!!
	frame[1].position[3] -= knee;
	//frame[1].footL.P=-frame[1].hipL.P-frame[1].kneeL.P;
	//frame[1].position[5] = +frame[1].position[1] + frame[1].position[4]; //OK!!!
	frame[1].position[5] = -frame[1].position[1] - frame[1].position[3];
	//Verificare che non azzeri le posizioni!!
	//frame[1].name.resize(8);
	//frame[1].position.resize(8);

	//frame[1].name.push_back("right_arm_joint_pitch");
	//frame[1].name.push_back("left_arm_joint_pitch");

	//frame[1].elbowR.P=elbow/2;
	frame[1].position[6]=elbow / 2;
	//frame[1].elbowL.P=0;
	frame[1].position[7]=0.0;

	//frame[1].name[6] ="right_arm_joint_pitch";
	//frame[1].name[7] ="left_arm_joint_pitch";


	//frame[1].position[6]=elbow/2;
	//frame[1].position[7]=0;

	//third position
	frame[2] = frame[1];
	//frame[2].kneeR.P+= ex_knee;
	frame[2].position[2] += ex_knee;
	//frame[2].footR.P+= ex_ankle;
	frame[2].position[4] += ex_ankle;
	//frame[2].hipR.P=-frame[2].footR.P-frame[2].kneeR.P;
	frame[2].position[0] = -frame[2].position[4] - frame[2].position[2];
	//frame[2].hipL.P=thighBase;
	frame[2].position[1] = thighBase;
	//frame[2].kneeL.P=kneeBase;
	frame[2].position[3] = kneeBase;
	//frame[2].footL.P=ankleBase;
	frame[2].position[5] = ankleBase;
	//frame[2].elbowR.P=elbow;
	frame[2].position[6] = elbow;
	//frame[2].elbowL.P=-elbow/2;
	frame[2].position[7] = -elbow / 2;

	//fourth position
	//frame[3]=frame[0];
	frame[3] = frame[0];
	//frame[3].hipR.P+=thigh;
	frame[3].position[0] += thigh;
	//frame[3].kneeR.P -=knee;
	frame[3].position[2] -= knee;
	//frame[3].footR.P=-frame[3].hipR.P-frame[3].kneeR.P;
	frame[3].position[4] = -frame[3].position[0] - frame[3].position[2];
	//frame[3].elbowR.P=0;
	frame[3].position[6]=0;
	 //frame[3].elbowL.P=elbow/2;
	frame[3].position[7]=elbow / 2;


	//fifth position
	frame[4] = frame[3];
	//frame[4].kneeL.P+= ex_knee;
	frame[4].position[3] += ex_knee;
	//frame[4].footL.P+= ex_ankle;
	frame[4].position[5] += ex_ankle;
	//frame[4].hipL.P=-frame[4].footL.P-frame[4].kneeL.P;
	frame[4].position[1] = -frame[4].position[5] - frame[4].position[3];
	//frame[4].hipR.P=thighBase;
	frame[4].position[0] = thighBase;
	//frame[4].kneeR.P=kneeBase;
	frame[4].position[2] = kneeBase;
	//frame[4].footR.P=ankleBase;
	frame[4].position[4] = ankleBase;
	//frame[4].elbowR.P=-elbow/2;
	frame[4].position[6] = -elbow / 2;
	//frame[4].elbowL.P=elbow;
	frame[4].position[7] = elbow;

	//frame swing sx
	frame[5] = joint_state;

	frame[5].name.push_back("right_hip_joint_pitch");
    frame[5].name.push_back("left_hip_joint_pitch");
	frame[5].name.push_back("right_knee_joint_pitch");
	frame[5].name.push_back("left_knee_joint_pitch");

	frame[5].name.push_back("right_ankle_joint_pitch");
	frame[5].name.push_back("left_ankle_joint_pitch");


	frame[5].name.push_back("right_shoulder_joint_pitch");
	frame[5].name.push_back("left_shoulder_joint_pitch");

	frame[5].name.push_back("right_hip_joint_roll");
	frame[5].name.push_back("right_ankle_joint_roll");
	frame[5].name.push_back("left_hip_joint_roll");
	frame[5].name.push_back("left_ankle_joint_roll");


	//frame[5].position.push_back(hip);
	//frame[5].position.push_back(-hip);
	//cambio segno 06.07.2015
	frame[5].position.push_back(-hip);
    frame[5].position.push_back(hip);
	//frame[5].position.push_back(0.0);
	//frame[5].position.push_back(0.0);

	frame[5].position.push_back(0.0);
	frame[5].position.push_back(0.0);

	//frame[5].position.push_back(ankle_r);
	//frame[5].position.push_back(-ankle_r);
	//cambio segno 06.07.2015
	frame[5].position.push_back(-ankle_r);
    frame[5].position.push_back(ankle_r);
	//frame[5].position.push_back(0.0);
	//frame[5].position.push_back(0.0);

	frame[5].position.push_back(0.0);
	frame[5].position.push_back(0.0);

	frame[5].position.push_back(-hip);
	frame[5].position.push_back(-ankle_r);
	frame[5].position.push_back(hip);
	frame[5].position.push_back(ankle_r);

	//frame[5].name[0] ="right_hip_joint_pitch";
	//frame[5].name[1] ="right_ankle_joint_pitch";
	//frame[5].name[2] ="left_hip_joint_pitch";
	//frame[5].name[3] ="left_ankle_joint_pitch";

	//frame[5].position[0] =   hip;
	//frame[5].position[1] =  ankle_r;
	//frame[5].position[2] =  -hip;
	//frame[5].position[3] =  -ankle_r;

	//frame swing dx
	frame[6] = joint_state;
	//frame[6].name.resize(4);
	//frame[6].position.resize(4);

	frame[6].name.push_back("right_hip_joint_pitch");
	frame[6].name.push_back("left_hip_joint_pitch");
    frame[6].name.push_back("right_knee_joint_pitch");
	frame[6].name.push_back("left_knee_joint_pitch");

	frame[6].name.push_back("right_ankle_joint_pitch");
	frame[6].name.push_back("left_ankle_joint_pitch");


	frame[6].name.push_back("right_shoulder_joint_pitch");
	frame[6].name.push_back("left_shoulder_joint_pitch");

	frame[6].name.push_back("right_hip_joint_roll");
	frame[6].name.push_back("right_ankle_joint_roll");
	frame[6].name.push_back("left_hip_joint_roll");
	frame[6].name.push_back("left_ankle_joint_roll");



	//frame[6].position.push_back(-hip);
	//frame[6].position.push_back(hip);
	//cambio segno 06.07.2015
	frame[6].position.push_back(hip);
	frame[6].position.push_back(-hip);
	//frame[6].position.push_back(0.0);
	//frame[6].position.push_back(0.0);

	frame[6].position.push_back(0.0);
	frame[6].position.push_back(0.0);

	//frame[6].position.push_back(-ankle_r);
	//frame[6].position.push_back(ankle_r);
	//cambio segno 06.07.2015
	frame[6].position.push_back(ankle_r);
	frame[6].position.push_back(-ankle_r);
    //frame[6].position.push_back(0.0);
    //frame[6].position.push_back(0.0);

	frame[6].position.push_back(0.0);
	frame[6].position.push_back(0.0);

	frame[6].position.push_back(hip);
	frame[6].position.push_back(ankle_r);
	frame[6].position.push_back(-hip);
	frame[6].position.push_back(-ankle_r);

	//frame[6].name[0] ="right_hip_joint_pitch";
	//frame[6].name[1] ="right_ankle_joint_pitch";
	//frame[6].name[2] ="left_hip_joint_pitch";
	//frame[6].name[3] ="left_ankle_joint_pitch";
	//frame[6].position[0] =  - hip;
	//frame[6].position[1] =  -ankle_r;
	//frame[6].position[2] =  hip;
	//frame[6].position[3] =  ankle_r;

}

//MODIFICADO HASTA ACA

//Invia le posizioni al robot reale/simulato
void RobovieWalkerEnv::sendPos(sensor_msgs::JointState msg) {
	//cout << "\n DENTRO sendPos n=" << msg.name.size() << "\n";
	//Dichiarazione publisher

	//ros::NodeHandle nh;
	//ros::Publisher pub;

	//pub = nh.advertise<sensor_msgs::JointState> ("joint_states", 1);
	//pub.publish(msg);
	//usleep(250*1000);

	//nuova gestione 2015
	sensor_msgs::JointState m;
	vector<double> values;
	vector<double> names;

	//m.name.push_back("left_shoulder_joint_roll");
	//m.position.push_back(-2.0);
	//ros::spinOnce();
	//ros::Duration(0.2).sleep();
	//usleep(10 * 1000);

	//m.position[0]=-1.2;
	//ros::spinOnce();
	//ros::Duration(0.2).sleep();
	//usleep(10 * 1000);
	for (uint i = 0; i < msg.position.size(); i++) {

		 //msg.position[i] = - msg.position[i];

		//if (msg.name[i] == "right_ankle_joint_pitch" or msg.name[i] == "left_ankle_joint_pitch" or msg.name[i] == "left_knee_joint_pitch" or msg.name[i] == "right_ankle_joint_pitch" )
		if (msg.name[i] == "left_knee_joint_pitch" or msg.name[i] == "right_knee_joint_pitch" )
		{
			//cout << "\n DENTRO sendPos DOPO PUBLISH nome:" << msg.name[i] << " pos:"
							//<< msg.position[i] << "\n";
			 msg.position[i] = - msg.position[i];

		}
		if (msg.name[i] == "left_hip_joint_pitch" or msg.name[i] == "right_hip_joint_pitch" )
				{
					//cout << "\n DENTRO sendPos DOPO PUBLISH nome:" << msg.name[i] << " pos:"
									//<< msg.position[i] << "\n";
					 msg.position[i] = - msg.position[i];

		}
		if (msg.name[i] == "left_hip_joint_roll" or msg.name[i] == "right_hip_joint_roll" )
						{
							//cout << "\n DENTRO sendPos DOPO PUBLISH nomeROLL:" << msg.name[i] << " pos:"
											//<< msg.position[i] << "\n";
							 msg.position[i] = - msg.position[i];

				}
		if (msg.name[i] == "left_ankle_joint_pitch" or msg.name[i] == "right_ankle_joint_pitch" )
						{
							//cout << "\n DENTRO sendPos DOPO PUBLISH nome:" << msg.name[i] << " pos:"
											//<< msg.position[i] << "\n";
							 msg.position[i] = - msg.position[i];

		}
		if (msg.name[i] == "left_ankle_joint_roll" or msg.name[i] == "right_ankle_joint_roll" )
								{
									//cout << "\n DENTRO sendPos DOPO PUBLISH nome:" << msg.name[i] << " pos:"
													//<< msg.position[i] << "\n";
									 msg.position[i] = - msg.position[i];

				}
		if (msg.name[i] == "left_shoulder_joint_pitch" )//or msg.name[i] == "right_shoulder_joint_pitch" )
								{
									//cout << "\n DENTRO sendPos DOPO PUBLISH nome:" << msg.name[i] << " pos:"
										//			<< msg.position[i] << "\n";
									 msg.position[i] = - msg.position[i];

		}

		//values.push_back(joints[i]->getJointValue());

		//m.position.push_back(hj.position[i]);
		//m.name.push_back(hj.name[i]);

		//m.name.push_back("left_shoulder_joint_pitch");
		//m.position.push_back(-1.2);

		//m.position[0]=-1.2;
		//m.name[0]="left_shoulder_joint_pitch";


		//cout << "\n DENTRO RESIZE NAME[0]=" << hj.name[0] << "\n";

		//cout << "\n DENTRO sendPos DOPO PUBLISH nome:" << msg.name[i] << " pos:"
			//	<< msg.position[i] << "\n";
	}

	//m.name. = names;
	//m.position = values;

	// pubblico
	//sub.publish(m);

	//pub.publish(m);
	//09.05.2015 Vediamo se invia i frame giusti

	//m.name.push_back("right_shoulder_joint_pitch");
	//m.position.push_back(-1.2);
	
	pub.publish(msg);
	//pub.publish(frame[1]);
	
	//ros::spinOnce ();
	//ros::Duration(0.03).sleep();
	usleep(10 * 1000);
	//cout << "\n ESCE sendPos";

	//usleep(10*1000);
	//cout << "\n DENTRO sendPos DOPO PUBLISH \n";

	//ros::spinOnce();
	//VEDERE SE MANCA SLEEP!
	//usleep(10*1000);
	//ros::spinOnce();
}

#define K 1
#define CYCLETIME 125000 / K

void RobovieWalkerEnv::executeStep() { // invia le posizioni del motore al robot reale o simulato

	//cout << "\n DENTRO executeStep \n";
	/*if (!running)
	 return;*/

	long t0, t1;
	t0 = timestamp;

	//Invia posizioni al robot
	sendPos(hj);

	//cout << "\n DENTRO executeStep DOPO sendPos \n";

#ifdef WIN32
	t1 = (GetTickCount () - startTime) * 1000;
#else
	struct timeval tv;
	gettimeofday(&tv, 0);
	t1 = tv.tv_sec * 1000000 + tv.tv_usec;
	//ros::Time time = ros::Time::now();
	//t1 = time.sec * 1000000+time.nsec;
#endif
	while (t1 - t0 < CYCLETIME) {
		usleep(min((long) 500, CYCLETIME - (t1 - t0)));
		//ros::Duration d = ros::Duration(min((long) 500, CYCLETIME - (t1 - t0))/1000, 0);
		//d.sleep();
#ifdef WIN32
		t1 = (GetTickCount () - startTime) * 1000;
#else
		gettimeofday(&tv, 0);
		t1 = tv.tv_sec * 1000000 + tv.tv_usec;
		//ros::Time time2 = ros::Time::now();
		//t1 = time2.sec * 1000000+time2.nsec;
#endif
	}
	//      printf("cycle time = %ld\n",t1-timestamp);
	timestamp = t1;

	age = (timestamp - time0) / 1000;
	nframes++;

	//cout << age <<" DENTRO executeStep DOPO sendPos \n";

	//VERIFICAR ESTE COMENTARIO -- fstable trovata solo nella classe URobot
	//fstable += fabs (lvel.x);
}

//Recupera la posizione del robot virtuale
bool RobovieWalkerEnv::getObjectPose(std::string model_name,
		geometry_msgs::Pose &pose) {//cout << "\n DENTRO getObjectPose \n";
	gazebo_msgs::GetModelState::Request req;
	req.model_name = model_name;
	gazebo_msgs::GetModelState::Response res;

	//ROS_INFO("getting %s state", model_name.c_str());
	int n_try = 3;
	while (nh_->ok()
			&& !ros::service::call("/gazebo/get_model_state", req, res)) {
		if (n_try == 0) {
			std::cerr << "couldn't get " << model_name << " state\n";
			return false;
		}

		ros::spinOnce();
		ros::Duration(0.1).sleep();
		n_try--;
	}

	pose = res.pose;
	return true;
}

//2015 - Recupera la posizione del robot virtuale
bool RobovieWalkerEnv::resetRobotGazebo(std::string model_name) {
	//09.05.2015 Prima bisogna resettare la posizione dei giunti
	//ros::NodeHandle nh;
	//ros::Publisher pub;
	//pub = nh.advertise<sensor_msgs::JointState> ("joint_states", 1);
	sensor_msgs::JointState m;
	// Spalla
	m.name.push_back("right_shoulder_joint_roll");
	m.name.push_back("right_shoulder_joint_pitch");
	m.name.push_back("left_shoulder_joint_roll");
	m.name.push_back("left_shoulder_joint_pitch");

	// Braccio
	m.name.push_back("right_arm_joint_pitch");
	m.name.push_back("left_arm_joint_pitch");

	// Testa
	m.name.push_back("head_joint_yaw");

	// Anca
	m.name.push_back("left_hip_joint_roll");
	m.name.push_back("left_hip_joint_pitch");
	m.name.push_back("right_hip_joint_roll");
	m.name.push_back("right_hip_joint_pitch");

	// Ginocchio
	m.name.push_back("left_knee_joint_pitch");
	m.name.push_back("right_knee_joint_pitch");

	// Caviglia
	m.name.push_back("left_ankle_joint_roll");
	m.name.push_back("left_ankle_joint_pitch");
	m.name.push_back("right_ankle_joint_roll");
	m.name.push_back("right_ankle_joint_pitch");


	for (uint i = 0; i < m.name.size(); i++) {
		//m.position[i] = 0;

		m.position.push_back(0.0);

	}

	//per separare i piedi
	//m.position[7]=0.67;
	//m.position[2]=0.57;

	m.position[7]=0.17;
    m.position[2]=0.17;
    m.position[0]=-0.17;

	pub.publish(m);

	//rimetto a zero
	//m.position[7]=0;
	//m.position[2]=0;
	usleep(10000 * 1000);
	//ros::Duration d = ros::Duration(1, 0);
	//d.sleep();
	//ros::spinOnce ();
	//ros::Duration (0.1).sleep ();
	//----

	gazebo_msgs::SetModelState::Request req;

	geometry_msgs::Pose posizioneIniziale;
	posizioneIniziale.position.x = 0.0;
	posizioneIniziale.position.y = 0.0;
	posizioneIniziale.position.z = 0.1;

	posizioneIniziale.orientation.x = 0.0;
	posizioneIniziale.orientation.y = 0.0;
	posizioneIniziale.orientation.z = 0.0;

	gazebo_msgs::ModelState statoModel;
	statoModel.model_name = "RobovieX";
	statoModel.pose = posizioneIniziale;

	statoModel.reference_frame = "world";

	//gazebo_msgs::GetModelState::Request req;
	req.model_state = statoModel;

	gazebo_msgs::SetModelState::Response res;

	//ROS_INFO("getting %s state", model_name.c_str());

	int n_try = 3;
	while (nh_->ok()
			&& !ros::service::call("/gazebo/set_model_state", req, res)) {
		if (n_try == 0) {
			std::cerr << "couldn't get " << model_name << " state\n";
			return false;
		}

		ros::spinOnce();
		//ros::Duration(0.1).sleep();
		usleep(10 * 1000);
		n_try--;
	}

	//resetta ancora - a volta non riesce a farlo per tutti i joint da terra
	pub.publish(m);
	//pub.publish(m);
	//pub.publish(m);
	//pub.publish(m);
	//pub.publish(m);
	//ros::Duration d2 = ros::Duration(3, 0);
	//d2.sleep();

	//per non camminare in volo
	usleep(3000 * 1000);

	return true;
}

//Resetta parametri controllo camminata
void RobovieWalkerEnv::resetWalk() {
	m_rightLegUp = m_leftLegUp = m_actionFinished = m_forceToFinish = false;
	m_stepNumber = 0;
}

//Calcola il prossimo frame
void RobovieWalkerEnv::executeAction(long time /*,HumanoidJoints &hj*/) {
	//cout << "\n DENTRO executeAction \n";
	// set the parameters range
	//  double shoulder=g.getMaskedGeneFloat(SHOULDER);
	double hip = hip_r_p; //g.getMaskedGeneFloat(HIP_R);
	double ankle_r = ankle_r_p;//g.getMaskedGeneFloat(ANKLE_R);
	// time
	int Time = time_p; //g.getMaskedGeneInt(TIME);
	// initial condinition

	int WAIT = 2000;
	sensor_msgs::JointState joint_state;

	/////////////////base

	if (time < WAIT) {
		//ROS_INFO("ENTRA0 %d",time);
		//cout << "\n ENTRA time < WAIT dimHj=" << hj.name.size() << "\n";
		//hj.reset();
		//hj.name.resize(6);
		//hj.position.resize(6);
		//cout << "\n ENTRA2 time < WAIT dimHj=" << hj.position.size() << "\n";
		/*
		 hj.name[0] ="right_hip_joint_pitch";
		 hj.name[1] ="left_hip_joint_pitch";
		 hj.name[2] ="right_knee_joint_pitch";
		 hj.name[3] ="right_ankle_joint_pitch";
		 hj.name[4] ="left_ankle_joint_pitch";
		 hj.name[5] ="left_knee_joint_pitch";

		 hj.position[1] = hj.position[0] = 0;
		 hj.position[5] = hj.position[2] = 0;
		 hj.position[4] = hj.position[3] = 0;
		 */

		hj=joint_state;
		hj.name.push_back("right_hip_joint_pitch");
		hj.name.push_back("left_hip_joint_pitch");
		hj.name.push_back("right_knee_joint_pitch");
		hj.name.push_back("left_knee_joint_pitch");
		hj.name.push_back("right_ankle_joint_pitch");
		hj.name.push_back("left_ankle_joint_pitch");


		hj.name.push_back("right_shoulder_joint_pitch");
		hj.name.push_back("left_shoulder_joint_pitch");

		hj.name.push_back("right_hip_joint_roll");
		hj.name.push_back("right_ankle_joint_roll");
		hj.name.push_back("left_hip_joint_roll");
		hj.name.push_back("left_ankle_joint_roll");

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);
		hj.position.push_back(0);
		hj.position.push_back(0);

		resetWalk();
		// the robot starts lowered
		double distance = m_legLength - (m_legLength - m_legLength * .75)
				* time / WAIT;// m_legLength*acc/1000.0;
		// calculate the cynematic chain angles
		double phi_d = 3 * M_PI / 2;
		double phi_B = phi_d + acos((m_legALength * m_legALength - m_legBLength
				* m_legBLength - distance * distance) / (2 * distance
				* m_legBLength));
		phi_B -= (phi_B > 2 * M_PI) ? 2 * M_PI : 0;
		// knee coordinates
		double x_knee = m_legBLength * cos(phi_B);
		double z_knee = m_legBLength * sin(phi_B);
		double phi_A = atan2((distance - z_knee), (-x_knee));

		//hj.name.resize(6);
		//hj.position.resize(6);
		/*
		 hj.name[0] ="right_hip_joint_pitch";
		 hj.name[1] ="left_hip_joint_pitch";
		 hj.name[2] ="right_knee_joint_pitch";
		 hj.name[3] ="right_ankle_joint_pitch";
		 hj.name[4] ="left_ankle_joint_pitch";
		 hj.name[5] ="left_knee_joint_pitch";

		 hj.position[1] = hj.position[0] = (-M_PI/2 + phi_A);
		 hj.position[5] = hj.position[2] = -(phi_A - phi_B);
		 hj.position[4] = hj.position[3] = M_PI/2-phi_B;
		 */
		/*
		hj.name.push_back("right_hip_joint_pitch");
		hj.name.push_back("left_hip_joint_pitch");
		hj.name.push_back("right_knee_joint_pitch");
		hj.name.push_back("right_ankle_joint_pitch");
		hj.name.push_back("left_ankle_joint_pitch");
		hj.name.push_back("left_knee_joint_pitch");

		hj.position.push_back((-M_PI / 2 + phi_A));
		hj.position.push_back((-M_PI / 2 + phi_A));

		hj.position.push_back(-(phi_A - phi_B));
		hj.position.push_back(M_PI / 2 - phi_B);

		hj.position.push_back(M_PI / 2 - phi_B);
		hj.position.push_back(-(phi_A - phi_B));
        */
		double bodyp=-M_PI/10;
		double hj_body_P= bodyp - bodyp*time/WAIT;

		 hj.position[1] = hj.position[0] = (-M_PI/2 + phi_A);
		 hj.position[3] = hj.position[2] = -(phi_A - phi_B);

	     hj.position[5] = hj.position[4] = M_PI/2-phi_B-hj_body_P;
		//hj.hipL.P=hj.hipR.P = (-M_PI/2 + phi_A);
		//hj.kneeL.P=hj.kneeR.P = -(phi_A - phi_B);
		//hj.footL.P=hj.footR.P = M_PI/2-phi_B;
		//double bodyp=-M_PI/10;
		//double hj_body_P= bodyp - bodyp*time/WAIT;
		m_state = 0;
		//cout << "\n ESCE executeAction m_state 0 \n";
		return;
	}

	//Decommentare per guardare solo il frame 0
	//return;
	time -= WAIT;

	//prova solo frame
	while (time > Time)
		time -= Time;

	int n_frame = 8;
	float interval = 1.0 * Time / 8;

	//hj.reset();
	sensor_msgs::JointState joint_state2;
	hj=joint_state2;
	//hj.name.resize(6);
	//hj.position.resize(6);
	/*
	 hj.name[0] ="right_hip_joint_pitch";
	 hj.name[1] ="left_hip_joint_pitch";
	 hj.name[2] ="right_knee_joint_pitch";
	 hj.name[3] ="right_ankle_joint_pitch";
	 hj.name[4] ="left_ankle_joint_pitch";
	 hj.name[5] ="left_knee_joint_pitch";

	 hj.position[1] = hj.position[0] = 0;
	 hj.position[5] = hj.position[2] = 0;
	 hj.position[4] = hj.position[3] = 0;
	 */
	hj.name.push_back("right_hip_joint_pitch");
	hj.name.push_back("left_hip_joint_pitch");
	hj.name.push_back("right_knee_joint_pitch");
	hj.name.push_back("left_knee_joint_pitch");
	hj.name.push_back("right_ankle_joint_pitch");
	hj.name.push_back("left_ankle_joint_pitch");

	hj.name.push_back("right_shoulder_joint_pitch");
	hj.name.push_back("left_shoulder_joint_pitch");

	hj.name.push_back("right_hip_joint_roll");
	hj.name.push_back("right_ankle_joint_roll");
	hj.name.push_back("left_hip_joint_roll");
	hj.name.push_back("left_ankle_joint_roll");

	hj.position.push_back(0);
	hj.position.push_back(0);
	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(0);
	hj.position.push_back(0);

	//hj = new sensor_msgs::JointState();

	if (time < interval) {
		//ROS_INFO("ENTRA1 %d",time);
		//hj+= (m_state==0)?frame[0]:frame[4];
		//hj+=frame[5]/2;

		if (m_state == 0) {
			for (uint i = 0; i < hj.position.size(); i++) {
				hj.position[i] = hj.position[i] + frame[0].position[i];
				//cout << "\n DENTRO executeAction frame1_1 pos[i]=" << hj.position[i] << " [i]=" << i << "\n";
			}
		} else {
			for (uint i = 0; i < hj.position.size(); i++) {
				hj.position[i] = hj.position[i] + frame[4].position[i];
				//cout << "\n DENTRO executeAction frame1_2 pos[i]=" << hj.position[i] << " [i]=" << i << "\n";
			}
		}
		//Decommentare per guardare solo il frame 0
	    //return;
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[5].position[i] / 2;
			//cout << "\n DENTRO executeAction frame5_1 pos[i]=" << hj.position[i] << "\n";
		}
		//cout << "\n ESCE executeAction m_state 00 \n";
		return;

	}
	//Decommentare per guardare solo il frame 1
    //return;
	if (time < 2 * interval) {
		//hj+=frame[5] + frame[1];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[5].position[i]
					+ frame[1].position[i];
			//cout << "\n DENTRO executeAction frame5_2 pos[i]=" << hj.position[i] << "\n";
		}

		//cout << "\n ESCE executeAction m_state 1 \n";
		m_state = 1;
		return;
	}
	//Decommentare per guardare solo il frame 2
	//return;
	if (time < 3 * interval) {
		//hj+=frame[5]/2 + frame[2];

		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[5].position[i] / 2
					+ frame[2].position[i];
			//cout << "\n DENTRO executeAction frame5_3 pos[i]=" << hj.position[i] << "\n";
		}
		//cout << "\n ESCE executeAction time < 3 \n";
		return;
	}
	//Decommentare per guardare solo il frame 3
    //return;
	if (time < 4 * interval) {
		//hj+= frame[2];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[2].position[i];
		}
		//cout << "\n ESCE executeAction time < 4 \n";
		return;
	}
	//Decommentare per guardare solo il frame 4
	//return;
	if (time < 5 * interval) {
		//hj+= frame[2] + frame[6]/2;
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[2].position[i]
					+ frame[6].position[i] / 2;
		}
		//cout << "\n ESCE executeAction time < 5 \n";
		return;
	}
	//Decommentare per guardare solo il frame 5
	//return;
	if (time < 6 * interval) {
		//hj+=frame[6] + frame[3];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[6].position[i]
					+ frame[3].position[i];
			//cout << "\n DENTRO executeAction frame6_1 pos[i]=" << hj.position[i] << "\n";
		}
		//cout << "\n ESCE executeAction time < 6 \n";
		return;
	}
	//Decommentare per guardare solo il frame 6
	//return;
	if (time < 7 * interval) {
		//hj+=frame[6]/2 + frame[4];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[6].position[i] / 2
					+ frame[4].position[i];
			//cout << "\n DENTRO executeAction frame6_2 pos[i]=" << hj.position[i] << "\n";
		}
		//cout << "\n ESCE executeAction time < 7 \n";
		return;
	}
	if (time < 8 * interval) {
		//hj+= frame[4];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[4].position[i];
		}
		//cout << "\n ESCE executeAction time < 8 \n";
		return;
	}

	//cout << "\n ESCE executeAction fuori if \n";
	return;

	//fine prova

	while (time > Time)
		time -= Time;

	float osc = sin(2 * M_PI * time / Time);
	osc = osc * osc * osc;

	//hj.reset();
	sensor_msgs::JointState joint_state3;
	hj=joint_state3;
	// oscillate on the coronal plane
	hip = hip * osc;
	ankle_r = ankle_r * osc;
	//hj.hipR.R =   hip;
	//hj.footR.R =  ankle_r;
	//hj.hipL.R =  -hip;
	//hj.footL.R =  -ankle_r;

	//hj.name.resize(10);
	//hj.position.resize(10);
	/*
	 hj.name[0] ="right_hip_joint_pitch";
	 hj.name[1] ="left_hip_joint_pitch";
	 hj.name[2] ="right_knee_joint_pitch";
	 hj.name[3] ="rnew sensor_msgs::JointState();ight_ankle_joint_pitch";
	 hj.name[4] ="left_ankle_joint_pitch";
	 hj.name[5] ="left_knee_joint_pitch";

	 hj.name[6] ="right_hip_joint_roll";
	 hj.name[7] ="right_ankle_joint_roll";
	 hj.name[6] ="left_hip_joint_roll";
	 hj.name[7] ="left_ankle_joint_roll";
new sensor_msgs::JointState();
	 hj.position[1] = hj.position[0] = 0;
	 hj.position[5] = hj.position[2] = 0;
	 hj.position[4] = hj.position[3] = 0;
	 hj.position[6] = hip;
	 hj.position[7] = ankle_r;
	 hj.position[8] = -hip;
	 hj.position[9] = -ankle_r;
	 */
	hj.name.push_back("right_hip_joint_pitch");
	hj.name.push_back("left_hip_joint_pitch");
	hj.name.push_back("right_knee_joint_pitch");
	hj.name.push_back("left_knee_joint_pitch");
	hj.name.push_back("right_ankle_joint_pitch");
	hj.name.push_back("left_ankle_joint_pitch");

	hj.name.push_back("right_shoulder_joint_pitch");
	hj.name.push_back("left_shoulder_joint_pitch");

	hj.name.push_back("right_hip_joint_roll");
	hj.name.push_back("right_ankle_joint_roll");
	hj.name.push_back("left_hip_joint_roll");
	hj.name.push_back("left_ankle_joint_roll");

	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(0);
	hj.position.push_back(0);

	hj.position.push_back(hip);
	hj.position.push_back(ankle_r);
	hj.position.push_back(-hip);
	hj.position.push_back(-ankle_r);

	//hj = new sensor_msgs::JointState();

	float threshold1 = .7;
	float threshold2 = .7;

	if (osc >= threshold1 || (osc > threshold2 && m_state == 1)) {
		//hj+= frame[1];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[1].position[i];
		}
		m_state = 1;
	} else if (osc <= threshold2 && osc > -threshold1 && m_state == 1) {
		//hj+= frame[2];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[2].position[i];
		}
	} else if (osc <= -threshold1 || (osc < -threshold2 && m_state == 2)) {
		//hj+= frame[3];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[3].position[i];
		}
		m_state = 2;
	} else if (osc >= -threshold2 && osc < threshold1 && m_state == 2) {
		//hj+= frame[4];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[4].position[i];
		}
	} else {
		//hj+=frame[0];
		for (uint i = 0; i < hj.position.size(); i++) {
			hj.position[i] = hj.position[i] + frame[0].position[i];
		}
	}
	//cout << "\n ESCE executeAction FONDO \n";
	return;

}

//Valutazione della fitness
double RobovieWalkerEnv::evaluateFitness(geometry_msgs::Pose &pose) {
	//cout << "\n DENTRO evaluateFitness \n";
	float x = (pose_ritorno.position.x > 0) ? pose_ritorno.position.x : 0;
	double score = x * x - fabs(pose_ritorno.position.y);
	if (score < 0)
		score = 0;
	score = sqrt(score);
	score = (pose_ritorno.position.z < 150) ? score : score + 500;

	return score;
}
//EvaluateWalk
float RobovieWalkerEnv::apply(int action) {
	int sec = 20 / K;
	double fstable_fin = 0.0;
	double height_f = 0.85;
	double score = 0;

	init();

	//action->init(genome,hj);
	//action->getMask(genome);

	while (age < 800 / K) {
		executeStep();
	}

	bool notCaduto = getObjectPose("RobovieX", pose_ritorno);

	//cout << "\n DENTRO evaluateFitness ORIENTATION=" << pose_ritorno.orientation.x << "\n";
	//cout << "\n DENTRO evaluateFitness ORIENTATION=" << pose_ritorno.orientation.y << "\n";
	//cout << "\n DENTRO evaluateFitness ORIENTATION=" << pose_ritorno.orientation.z << "\n";

	//2015 - la condizione di caduto scatta quando ho pose_ritorno.orientation = 0 nei tre assi - per gazebo
	if (pose_ritorno.orientation.x == 0.0 && pose_ritorno.orientation.y == 0.0
			&& pose_ritorno.orientation.z == 0.0) {
		cout << "\n DENTRO notCaduto";
		notCaduto = false;
	}

	// finisce il ciclo quando sono trascorsi i 20 secondi o quando il robot e' caduto
	//while ((!caduto || tipoSimulazione ) && pose_ritorno.position.x > 150 && age < sec * 1000 /*&& !action->actionFinished()*/) //    time limit in sec.
	while ((notCaduto) && age < sec * 1000 /*&& !action->actionFinished()*/) //    time limit in sec.
	{

		//VEDERE COME IMPLEMENTARE IL FINISH CON L'ALGORITMO DI RL
		//if (age>19*1000) action->forceToFinish();

		//09.05.2015 entra y funciona
		//cout << "\n DENTRO CICLO evaluateFitness WHILE / AGE=" << age << "\n";

		//Imposta il frame
		executeAction(age);

		//Invia i frame al robot
		executeStep();

	}
	if (tipoSimulazione) {
		do {
			printf(
					"Please,Insert x , y (mm) and Z(<150 if the robot is fallen down\n");
		}

		while (scanf("%f %f %f", &xfin, &yfin, &zfin) < 2);
		pose_ritorno.position.x = xfin;
		pose_ritorno.position.y = yfin;
		pose_ritorno.position.z = zfin;
	}
	else
	{
		getObjectPose("RobovieX", pose_ritorno);
	}

	//ROS_INFO("Tempo: %d\n", age);

	/*
	 if (model==kondo && (connection==UsarSim || connection==Both)){
	 hs.Position.lin.X=-hs.Position.lin.X; // il kondo e' storto!!!!!!!
	 }
	 */
	score = evaluateFitness(pose_ritorno);

	//ROS_INFO("Score: %.1f\n", score);

	return score;
}

//Per adesso completamente aleatorio, ma POSSIBILE CAMBIAMENTO
//Sarebbe un INIT?? SEMBRA INIT DEL'ALGORITMO DI RL - PRIME ITERAZIONI
std::vector<experience> RobovieWalkerEnv::getSeedings() {
	//cout << "\n DENTRO getSeedings \n";
	// return seedings
	std::vector<experience> seeds;
	//return seeds;

	//MANCA IMPLEMENTAZIONE DEL RESET!!
	reset();

	// some completely random (non target) -- PARTENZA CON VALORI BASSI
	//PROVO CON IL CICLO A 10 -- TERMINAR
	for (int i = 0; i < 10; i++) {

		//USA VARIABILI ALEATORIE (UNIFORME) - PARTE DA VALORI BASSI - RANGE RIDOTTO
		/*
		float time = rng.uniform(100, 200);
		float hip_r = rng.uniform(0.0525, 0.0625);
		float thigh = rng.uniform(0.0525, 0.0625);
		float knee = rng.uniform(0.0525, 0.0625);
		float ex_knee = rng.uniform(0.0525, 0.0625);
		float ex_ankle = rng.uniform(0.0525, 0.0625);
		float body = rng.uniform(0.0525, 0.0625);
		float ankle_r = rng.uniform(0.0525, 0.0625);
		float shoulder = rng.uniform(0.0525, 0.0625);
		*/


		//USA VARIABILI ALEATORIE (UNIFORME) - RANGE NORMALE
		/*
		 float   time = rng.uniform(100, 200);
		 float   hip_r = rng.uniform(0.525, 0.625);
		 float   thigh = rng.uniform(0.525, 0.625);
		 float   knee = rng.uniform(0.525, 0.625);
		 float   ex_knee = rng.uniform(0.525, 0.625);
		 float   ex_ankle = rng.uniform(0.525, 0.625);
		 float   body = rng.uniform(0.525, 0.625);
		 float   ankle_r = rng.uniform(0.525, 0.625);
		 float   shoulder = rng.uniform(0.525, 0.625);
		*/

		float time = rng.uniform(100, 200);
		float hip_r = rng.uniform(0.0525*7, 0.0625*7);
				float thigh = rng.uniform(0.0525*7, 0.0625*7);
				float knee = rng.uniform(0.0525*7, 0.0625*7);
				float ex_knee = rng.uniform(0.0525*7, 0.0625*7);
				float ex_ankle = rng.uniform(0.0525*7, 0.0625*7);
				float body = rng.uniform(0.0525*7, 0.0625*7);
				float ankle_r = rng.uniform(0.0525*7, 0.0625*7);
				float shoulder = rng.uniform(0.0525*7, 0.0625*7);
				
				
		experience experiencia = getExp(time, hip_r, thigh, knee, ex_knee, ex_ankle,
				body, ankle_r, shoulder, 1);
				
		cout << "\n DENTRO ::getSeedings reward:"<<experiencia.reward<<"\n";

		seeds.push_back(experiencia);
				
		//nuevo reset - camina? 2016
		reset();
	}

	//Perchè due volte reset?? OK calcola la partenza e poi reset per iniziare
	reset();

	return seeds;
}

void RobovieWalkerEnv::getMinMaxReward(float *minR, float *maxR) {
	//cout << "\n DENTRO getMinMaxReward \n";

	*minR = -120.0;
	*maxR = 100000.0;

}

//Qua posso verificare se il robot è giù
bool RobovieWalkerEnv::terminal() const {
	//cout << "\n DENTRO terminal " << pose_ritorno.position.y << "\n";
	if (pose_ritorno.position.y > 0.1) {
		return false;
	} else {
		return true;
	}
}

void RobovieWalkerEnv::reset() {
	//cout << "time_p=" << time_p  << "\n reset \n";
	/*
	 time_p = 0.0;
	 hip_r_p= 0.0;
	 thigh_p= 0.0;
	 knee_p= 0.0;
	 ex_knee_p= 0.0;
	 ex_ankle_p= 0.0;
	 body_p= 0.0;
	 ankle_r_p= 0.0;
	 shoulder_p= 0.0;
	 */
	nh_ = new ros::NodeHandle();

	bool resettaRobot = resetRobotGazebo("RobovieX");

	//DA INIT() URobot.cpp
	/*
	 struct timeval tv;
	 gettimeofday (&tv, 0);
	 timestamp = tv.tv_sec * 1000000 + tv.tv_usec;
	 */
	//cout << "time_p=" << timestamp  << "\n reset \n";

	#ifdef WIN32
		startTime = GetTickCount ();
		//cout << "startTime=" << startTime  << "\n reset \n";
	#endif

	#ifdef WIN32
		timestamp = (GetTickCount () - startTime) * 1000;
	#else
		struct timeval tv;
		gettimeofday(&tv, 0);
		timestamp = tv.tv_sec * 1000000 + tv.tv_usec;

		//ros::Time time2 = ros::Time::now();
		//timestamp = time2.sec * 1000000+time2.nsec;
	#endif

	time0 = timestamp;
	age = 0;
	steps = 0;
	nframes = 0;
	fstable = 0.0;

	init();

	//Questo init va con le posizioni iniziali
	//hj.name.resize(6);
	//hj.position.resize(6);
	/*
	 hj.name[0] ="right_hip_joint_pitch";
	 hj.name[1] ="left_hip_joint_pitch";
	 hj.name[2] ="right_knee_joint_pitch";
	 hj.name[3] ="right_ankle_joint_pitch";
	 hj.name[4] ="left_ankle_joint_pitch";
	 hj.name[5] ="left_knee_joint_pitch";

	 hj.position[1] = hj.position[0] = 0;
	 hj.position[5] = hj.position[2] = 0;
	 hj.position[4] = hj.position[3] = 0;
	 */

	sensor_msgs::JointState joint_state3;
		hj=joint_state3;
		hj.name.push_back("right_hip_joint_pitch");
		hj.name.push_back("left_hip_joint_pitch");
		hj.name.push_back("right_knee_joint_pitch");
		hj.name.push_back("left_knee_joint_pitch");
		hj.name.push_back("right_ankle_joint_pitch");
		hj.name.push_back("left_ankle_joint_pitch");

		hj.name.push_back("right_shoulder_joint_pitch");
		hj.name.push_back("left_shoulder_joint_pitch");

		hj.name.push_back("right_hip_joint_roll");
		hj.name.push_back("right_ankle_joint_roll");
		hj.name.push_back("left_hip_joint_roll");
		hj.name.push_back("left_ankle_joint_roll");

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);

		hj.position.push_back(0);
		hj.position.push_back(0);
		hj.position.push_back(0);
		hj.position.push_back(0);

}

