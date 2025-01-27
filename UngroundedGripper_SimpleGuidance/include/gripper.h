#pragma once
#include "chai3d.h"
#include "pantograph.h"


#include <cmath>
#include <array>
#include <cstdlib>

using namespace chai3d;
using namespace std;

#define NUM_ENC 5	// nunber of encoders 2 on each finger and gripper motor = 5
#define NUM_MTR 5 

enum { indexDist, indexProx, thumbProx, thumbDist, gripMotor };

////------------------------------------------------------------------------------
//// ERROR CONSTANTS
//const bool C_ERROR = false;			//! Function returns with an error.
//const bool C_SUCCESS = true;		//! Function returns successfully.
////------------------------------------------------------------------------------


class gripper : public cGenericHapticDevice
{

private:

public:

	pantograph pThumb;
	pantograph pIndex;

	cVector3d m_force;
	cVector3d m_torque;
	double m_gripForce;
	double m_gripForce_last = 0;
	cVector3d m_thumbForce;
	cVector3d m_fingerForce;
	

	bool m_error;                           // TRUE = problem with exo while running
	std::string m_errMessage;               // error message
	double m_t;                             // current time [sec]
	chai3d::cPrecisionClock* m_clk;         // pointer to clock for computing velocity

	bool m_gripperAvailable;
	bool m_gripperReady;
	chai3d::cMutex m_gripperLock;

	vector<double> m_thZero;                  // zero angles for motor-angle measurement [counts, in motor space]
	//vector<double> startAngle = { 0.0, 0.0, 0.0, 0.0, PI / 4 }; // values at the zero position


	vector<double> m_th;				//	motor angles [rad]
	vector<double> m_thDes;			// desired motor angles [rad]
	vector<double> m_thErr;              // joint-angle error [rad]	
	vector<double> m_thdot;              // current joint velocities [rad/s]
	vector<double> m_thdotDes;           // desired joint velocities [rad/s]
	vector<double> m_thdotErr;           // joint-velocity error [rad/s]
	vector<double> m_thErrInt;           // integrated joint angle error [rad*s]
	vector<double> m_T;		// motor torques to command 
	const vector<double> m_Kp = { 0.3,  0.3,  0.3,  0.3, 3 }; //{ 0.6,  0.6,  0.6,  0.6, 3 };  // [N/rad]
	const vector<double> m_Kd = { 0.001, 0.001, 0.001, 0.001, 1 }; //{ 0.00001, .00001, 0.00001, .00001, 0.1 };
	const vector<double> m_Ki = {0.000, 0.000, 0.000, 0.000, 1}; //{ 0.01, 0.01, 0.01, 0.01, 0.01 };


	gripper(); // : pThumb(fingers::thumb), pIndex(fingers::index) { }
	~gripper();
	bool connect();
	bool disconnect();
	bool disableCtrl();
	bool calibrate();
	void getState();

	chai3d::cVector3d force;
	chai3d::cVector3d torque;
	double gripforce;


	void setForcesAndTorques(cVector3d a_force, cVector3d a_torque, double a_gripForce, cVector3d a_thumbForce, cVector3d a_fingerForce);
	void motorLoop(void);

private:
	double gripperLength;
};