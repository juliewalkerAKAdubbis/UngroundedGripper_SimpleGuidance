#pragma once

#include "chai3d.h"
#include "motorcontrol.h"
#include "gripper.h"
#include "gripperChaiDevice.h"
#include "magtracker.h"
#include "experiment.h"
#include "Windows.h"
#include <cmath>
#include <array>
#include <stdio.h>
#include <time.h>
#include <GLFW/glfw3.h>
#include "CODE.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace chai3d;
using namespace std;


#define RANGE_10V 0x00 // Range code for ADC �10V range.
#define RANGE_5V 0x10 // Range code for ADC �5V range.
#define EOPL 0x80 // ADC end-of-poll-list marker.
#define CHANMASK 0x0F // ADC channel number mask.

// non member prototypes
void errorCallback(int a_error, const char* a_description);


class hapticsThread
{
public:


	// initialization functions
	int initialize(void);
	int initializeChai3dStuff(void);
	int setUpWorld(void);
	int setUpHapticDevice(void);
	void setUpWidgets(void);

	// running functions
	void updateHaptics(void);
	void updateGraphics(void);
	

	//------------------------------------------------------------------------------
	// EXPERIMENT VARIABLES AND FUNCTIONS
	//------------------------------------------------------------------------------
	experiment Exp; 
	bool runningExperiment = true;
	chai3d::cPrecisionClock exp_clk;         // pointer to clock for experiment 
	chai3d::cPrecisionClock record_clk;		// clock for recording data
	double exp_t;                             // current time [sec]
	double exp_t_start;						// time at start of cue [sec]
	double m_cueDuration = 0.5;		// [s]
	double m_returnDuration = 3.0;	// [s]
	int m_cue;
	double m_magnitude;
	cVector3d thumbForce_start = { 0.0, 0.0, 0.0 };
	cVector3d fingerForce_start = { 0.0, 0.0, 0.0 };

	std::ofstream file;
	
	void initRecording(int s);
	void record(void);

	//------------------------------------------------------------------------------
	// GENERAL SETTINGS
	//------------------------------------------------------------------------------
	cStereoMode stereoMode = C_STEREO_DISABLED;
	// fullscreen mode
	bool fullscreen = false;
	// mirrored display
	bool mirroredDisplay = false;

	GLFWwindow* window = NULL;


	// rendering option
	bool showTexture = true;
	bool showNormals = false;
	bool showWireMode = false;

	//------------------------------------------------------------------------------
	// THREADING
	//------------------------------------------------------------------------------

	chai3d::cThread m_thread;
	chai3d::cMutex m_worldLock;
	chai3d::cMutex m_runLock;

	bool simulationRunning = true;			//simulation status
	bool simulationFinished = false;
	bool checkSimulationStatus(void);


	chai3d::cFrequencyCounter graphicRate;  // counter for graphics updates
	chai3d::cFrequencyCounter hapticRate;   // counter for haptics updates

	clock_t* m_timer;						// timer for graphics updates

	cFontPtr font;				// a font for rendering text
	cLabel* labelTrial;			// a label to display the rate [Hz] at which the simulation is running
	cLabel* labelStatus;			// a label to display the force on the gripper tool

//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

	cODEWorld* ODEWorld;

	// ODE objects
	cODEGenericBody* ODEBody0;
	cODEGenericBody* ODEBody1;
	cODEGenericBody* ODEBody2;

	// ODE objects
	cODEGenericBody* ODEGPlane0;
	cODEGenericBody* ODEGPlane1;
	cODEGenericBody* ODEGPlane2;
	cODEGenericBody* ODEGPlane3;
	cODEGenericBody* ODEGPlane4;
	cODEGenericBody* ODEGPlane5;

	//---------------------------------------------------------------------------
	// OBJECTS
	//---------------------------------------------------------------------------
				/// ------ TO DO: GET RID OF OBJECTS -----------
	double workspaceScaleFactor;

	// virtual objects
	cShapeCylinder* cylinder;
	cShapeSphere* m_curSphere0;
	cShapeSphere* m_curSphere1;
	cShapeLine* graspLine;
	cMesh* ground;
	cShapeBox* m_gripperBase;

	//---------------------------------------------------------------------------
	// CHAI3D World
	//---------------------------------------------------------------------------

	chai3d::cWorld* world;                  // CHAI world
	chai3d::cCamera* camera;                // camera to render the world
	chai3d::cSpotLight* light;       // light to illuminate the world
									 // a handle to window display context



	int width;                              // width of view
	int height;                             // height of view
	int mouseX;                             // cursor X-position
	int mouseY;                             // cursor Y-position


	chai3d::cGenericHapticDevicePtr chaiMagDevice;

	// tracker rotation variables
	chai3d::cVector3d m_position; 
	chai3d::cMatrix3d m_rotation;
	//chai3d::cMatrix3d fingerRotation0; chai3d::cMatrix3d deviceRotation0;

	cHapticDeviceHandler* handler;			// a haptic device handler
	cGenericHapticDevicePtr hapticDevice;	// a pointer to the current haptic device
	cToolCursor* tool;						// a virtual tool representing the haptic device in the scene
	gripper* m_gripper;						// the member gripper instantiation
	cMatrix3d m_gripperRot;
	cMultiMesh* finger;
	cMultiMesh* thumb;

	void pairWithGripper(gripper *a_gripper);
	void loadFingerMeshes();
	double toolRadius;
	cVector3d m_gripperForce;			// ---- TO DO: Streamline this ------
	cVector3d m_gripperTorque;
	double m_gripperGripForce;
	cVector3d m_thumbForce;
	cVector3d m_fingerForce;

	int swapInterval = 1;			// swap interval for the display context (vertical synchronization)

	//protected:


};