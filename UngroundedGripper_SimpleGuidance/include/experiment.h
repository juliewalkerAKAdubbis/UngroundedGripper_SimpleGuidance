#pragma once
#include "chai3d.h"
#include <time.h>
#include <string>
#include <vector>
//#include "hapticsThread.h"


#define MAX_TRIALS 32			// 80 (8 cues x 2 magnitudes x 5 repetitions)

using namespace chai3d;
using namespace std;

const double b = 0.25;	// small magnitude F [N]
const double s = 0.15;	// large magnitude F [N]

enum { FORWARD, BACKWARD, UP, DOWN, TWIST_LEFT, TWIST_RIGHT, TILT_LEFT, TILT_RIGHT };

const int cues[3][MAX_TRIALS] = { {0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7 },			// standard order
								  {6, 3, 0, 4, 5, 2, 1, 7, 4, 2, 7, 5, 6, 0, 1, 3, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7 } };		// randomized order
							
const double magnitudes[3][MAX_TRIALS] = { { b, b, b, b, b, b, b, b, s, s, s, s, s, s, s, s, b, b, b, b, b, b, b, b, s, s, s, s, s, s, s, s, },			// standard magnitude order
											{s, b, b, b, b, s, s, s, b, b, b, s, s, b, s, s, b, b, b, b, b, b, b, b, s, s, s, s, s, s, s, s, } };		// randomized order


class experiment
{
public:

	int m_trial;
	int m_subjNum;

	enum ExpStates{startingCue, applyingCue, startingReturn, returningToCenter };
	ExpStates state = startingCue;
	vector <string> cue_type = { "Forward", "Backward", "Up", "Down", "Twist Left", "Twist Right", "Tilt Left", "Tilt Right" };

	experiment();
	~experiment();
	void init();
	void getCue(int &a_cue, double &a_magnitude);
	void getCueForce(int a_cue, double a_magnitude, double m_t, double m_t_start, cVector3d &fingerForce, cVector3d &thumbForce, double cueDuration);
	void getReturnForce(double m_t, double m_t_start, cVector3d fingerForce_start, cVector3d thumbForce_start,cVector3d &fingerForce, cVector3d &thumbForce, double returnDuration);


	//// device, pair with haptics thread
	//hapticsThread* m_hapticsThread;
	//void pairWithHapticsThread(hapticsThread *a_hapticsThread);
	////void pairWithGripper(gripper *a_gripper);

private:


};