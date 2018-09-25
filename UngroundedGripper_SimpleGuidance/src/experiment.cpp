#include "experiment.h"

experiment::experiment() {
	m_trial = 0;

}

experiment::~experiment() {


}

void experiment::init() {
	cout << "Input subject number: " << endl;
	cin >> m_subjNum;
	cout << "Experiment Ready" << endl;
	m_trial = 0;
}


void experiment::getCue(int &a_cue, double &a_magnitude) {

		a_cue = cues[m_subjNum][m_trial];
		a_magnitude = magnitudes[m_subjNum][m_trial];
		cout << "Trial: " << m_trial << "     Cue: " << a_cue <<  "      Magnitude:" << a_magnitude << endl;



		// if (m_trial == MAX_TRIALS) {
		// 	m_hapticsThread->simulationFinished = true;
		// 	close();
		// }
		// else {
		// 	m_trial++;
		// }
	
}

void experiment::getCueForce(int a_cue, double a_magnitude, double m_t, double m_t_start, cVector3d &fingerForce, cVector3d &thumbForce, double cueDuration) {		
	double force = a_magnitude*(m_t - m_t_start) / (cueDuration);			// ---- TO DO: remove clock variables from EXP struct ---------
		
		switch (a_cue) {
		case FORWARD:
				fingerForce.set(0.0, 0.0, force);
				thumbForce.set(0.0, 0.0, force);
			break;

		case BACKWARD:
				fingerForce.set(0.0, 0.0, -force);
				thumbForce.set(0.0, 0.0, -force);
			break;									

		case UP:
				fingerForce.set(force, 0.0, 0.0);
				thumbForce.set(force, 0.0, 0.0);
			break;	

		case DOWN:
				fingerForce.set(-force, 0.0, 0.0);
				thumbForce.set(-force, 0.0, 0.0);
			break;

 		case TILT_LEFT:
 			fingerForce.set(-force, 0.0, 0.0);
 			thumbForce.set(force, 0.0, 0.0);
 			break;

 		case TILT_RIGHT:
 			fingerForce.set(force, 0.0, 0.0);
 			thumbForce.set(-force, 0.0, 0.0);
		break;	

 		case TWIST_LEFT:
 			fingerForce.set(0.0, 0.0, -force);
 			thumbForce.set(0.0, 0.0, force);
 			break;

 		case TWIST_RIGHT:
 			fingerForce.set(0.0, 0.0, force);
 			thumbForce.set(0.0, 0.0, -force);
 			break;

		default:
			break;
		}
	}

	// reset
	void experiment::getReturnForce( double m_t, double m_t_start, cVector3d fingerForce_start, cVector3d thumbForce_start, cVector3d &fingerForce, cVector3d &thumbForce, double returnDuration) {			
	cVector3d fingerIncrement; 
	cVector3d thumbIncrement;
	cVector3d fingerResultVec;
	cVector3d thumbResultVec;

		fingerForce_start.mulr((m_t - m_t_start) / returnDuration, fingerIncrement);
		fingerForce_start.subr(fingerIncrement, fingerResultVec);
		thumbForce_start.mulr((m_t - m_t_start) / returnDuration, thumbIncrement);
		thumbForce_start.subr(thumbIncrement, thumbResultVec);

		fingerForce = fingerResultVec;
		thumbForce = thumbResultVec;
	}



