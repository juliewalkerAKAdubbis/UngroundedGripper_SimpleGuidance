#pragma once
#include "chai3d.h"
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include "hapticsThread.h"


#define EXPERIMENT
# define MAX_TRIALS 10

using namespace chai3d;
using namespace std;

const double b = 0.4;	// small magnitude F [N]
const double s = 0.2;	// large magnitude F [N]

enum { FORWARD, BACKWARD, UP, DOWN, TWIST_LEFT, TWIST_RIGHT, TILT_LEFT, TILT_RIGHT };

const int cues[2][MAX_TRIALS] = { {0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },			// pseudorandom cue order
								  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1 } };

const double magnitudes[2][MAX_TRIALS] = { { b, b, b, b, b, s, s, s, s, s  },			// pseudorandom magnitude order
											{ b, b, b, b, b, s, s, s, s, s } };


class experiment
{
public:
	double m_t;                             // current time [sec]
	chai3d::cPrecisionClock m_clk;         // pointer to clock for computing velocity


	int m_trial;
	int m_subjNum;
	cVector3d m_pos;
	cMatrix3d m_rot;

	experiment();
	~experiment();
	void init();
	void runExperiment();


	// device, pair with haptics thread
	hapticsThread* m_hapticsThread;
	void pairWithHapticsThread(hapticsThread *a_hapticsThread);
	//void pairWithGripper(gripper *a_gripper);

private:
	double m_cueDuration = 0.5;
	double m_returnDuration = 2.0;
	int m_cue;
	double m_magnitude;
	std::ofstream file;


	void close();
	void openNewFile(int trial);
	void applyCue(int a_cue, double a_magnitude);

};