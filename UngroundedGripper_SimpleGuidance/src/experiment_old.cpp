#include "experiment.h"

experiment::experiment() {
	m_trial = 0;
	// set up file to record
	file.open("../UserData.txt");


}

experiment::~experiment() {


}

void experiment::init() {
	cout << "Input subject number: " << endl;
	cin >> m_subjNum;
	cout << "Experiment Ready" << endl;
	m_clk.start(true);
	m_trial = 0;
}

void experiment::pairWithHapticsThread(hapticsThread *a_hapticsThread) {
	m_hapticsThread = a_hapticsThread;
}

//void pairWithGripper(gripper *a_gripper) {
//	m_gripper = a_gripper;
//}


void experiment::runExperiment() {

	while (m_trial <= MAX_TRIALS && !m_hapticsThread->simulationFinished) {
		m_cue = cues[m_subjNum][m_trial];
		m_magnitude = magnitudes[m_subjNum][m_trial];

		// apply cue
		m_hapticsThread->m_runLock.acquire();
		cout << "Starting cue " << m_trial << endl;
		m_hapticsThread->m_runLock.release();
		applyCue(m_cue, m_magnitude);

		// record 
		// trial
		// action
		// xyz start position 
		// prq start orientation
		// xyz end position
		// prq end position

		// save file with whole trajectory
		// motor angle trajectory
		// magnetic tracker xyz
		// magnetic tracker pqr

		if (m_trial == MAX_TRIALS) {
			m_hapticsThread->simulationFinished = true;
			close();
		}
		else {
			m_trial++;
		}
	}
}

void experiment::applyCue(int a_cue, double a_magnitude) {
	m_t = m_clk.getCurrentTimeSeconds();
	double m_t_start = m_t;
	double force = 0.0;
	while ((m_t-m_t_start) < m_cueDuration) {
		//m_pos = m_hapticsThread->tool->getGlobalPos();
		//m_rot = m_hapticsThread->tool->getLocalRot();
		m_hapticsThread->m_runLock.acquire();
		m_pos = m_hapticsThread->position;
		m_rot = m_hapticsThread->rotation;
		m_hapticsThread->m_runLock.release();

		file << m_trial << ", " << a_cue << ", " << m_t << ", ";
		file << m_pos.x() << ", " << m_pos.y() << ", " << m_pos.z() << ", ";
		file << m_rot.getRow(0) << ", " << m_rot.getRow(1) << ", " << m_rot.getRow(2) << ", ";
		file << endl;

		cout << "test val:" << m_hapticsThread->testHapticsThreadConnection() << endl;
		
		switch (a_cue) {
		case FORWARD:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
				m_hapticsThread->m_fingerForce.set(0.0, 0.0, force);
				m_hapticsThread->m_thumbForce.set(0.0, 0.0, force);
				m_hapticsThread->m_runLock.release();

			break;
		case BACKWARD:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
			m_hapticsThread->m_fingerForce.set(0.0, 0.0, -force);
			m_hapticsThread->m_thumbForce.set(0.0, 0.0, -force);
			m_hapticsThread->m_runLock.release();
			break;

		case UP:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
			m_hapticsThread->m_fingerForce.set(force, 0.0, 0.0);
			m_hapticsThread->m_thumbForce.set(force, 0.0, 0.0);
			m_hapticsThread->m_runLock.release();

			break;
		case DOWN:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
			m_hapticsThread->m_fingerForce.set(-force, 0.0, 0.0);
			m_hapticsThread->m_thumbForce.set(-force, 0.0, 0.0);
			m_hapticsThread->m_runLock.release();
			break;

		case TILT_LEFT:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
			m_hapticsThread->m_fingerForce.set(-force, 0.0, 0.0);
			m_hapticsThread->m_thumbForce.set(force, 0.0, 0.0);
			m_hapticsThread->m_runLock.release();

			break;
		case TILT_RIGHT:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
			m_hapticsThread->m_fingerForce.set(force, 0.0, 0.0);
			m_hapticsThread->m_thumbForce.set(-force, 0.0, 0.0);
			m_hapticsThread->m_runLock.release();
			break;

		case TWIST_LEFT:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
			m_hapticsThread->m_fingerForce.set(0.0, 0.0, -force);
			m_hapticsThread->m_thumbForce.set(0.0, 0.0, force);
			m_hapticsThread->m_runLock.release();

			break;
		case TWIST_RIGHT:
			force = a_magnitude*(m_t - m_t_start) / (m_cueDuration);
			m_hapticsThread->m_runLock.acquire();
			m_hapticsThread->m_fingerForce.set(0.0, 0.0, force);
			m_hapticsThread->m_thumbForce.set(0.0, 0.0, -force);
			m_hapticsThread->m_runLock.release();
			break;

		default:
			break;
		}

	m_t = m_clk.getCurrentTimeSeconds();
	}

	// reset
	m_t = m_clk.getCurrentTimeSeconds(); m_t_start = m_t;
	cVector3d fingerForce_start(m_hapticsThread->m_fingerForce);
	cVector3d thumbForce_start(m_hapticsThread->m_thumbForce);
	cVector3d fingerIncrement; 
	cVector3d thumbIncrement;
	cVector3d fingerResultVec;
	cVector3d thumbResultVec;

	while ((m_t - m_t_start) < m_returnDuration) {
		m_t = m_clk.getCurrentTimeSeconds();
		fingerForce_start.mulr((m_t - m_t_start) / m_returnDuration, fingerIncrement);
		fingerForce_start.subr(fingerIncrement, fingerResultVec);
		thumbForce_start.mulr((m_t - m_t_start) / m_returnDuration, thumbIncrement);
		thumbForce_start.subr(thumbIncrement, thumbResultVec);

		m_hapticsThread->m_runLock.acquire();
		m_hapticsThread->m_fingerForce = fingerResultVec;
		m_hapticsThread->m_thumbForce = thumbResultVec;
		m_hapticsThread->m_runLock.release();
	}
	m_hapticsThread->m_runLock.acquire();
	cout << "RETURNING TO CENTER" << endl;
	m_hapticsThread->m_runLock.release();

}


void experiment::openNewFile(int trial) {


}

void experiment::close() {
	m_clk.stop();
	cout << "DONE WITH EXPERIMENT" << endl;
	m_hapticsThread->m_fingerForce.set(0.0, 0.0, 0.0);
	m_hapticsThread->m_thumbForce.set(0.0, 0.0, 0.0);
}
