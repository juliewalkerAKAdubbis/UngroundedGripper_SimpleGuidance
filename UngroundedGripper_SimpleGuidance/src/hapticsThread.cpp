#include "hapticsThread.h"		

// call in main before starting thread 
void hapticsThread::pairWithGripper(gripper *a_gripper) {
	m_gripper = a_gripper;

}

int hapticsThread::initialize( void ) {
	int errors = 0;
	
	// create Chai device for magnetic tracker
	chaiMagDevice = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::gripperChaiDevice( 0)));

	initializeChai3dStuff();
	m_worldLock.acquire();
	setUpWorld();
	setUpWidgets();
	setUpHapticDevice();
	m_worldLock.release();
	
	// set output to zero to start
	m_gripperForce = { 0.0, 0.0, 0.0 };
	m_gripperTorque = { 0.0, 0.0, 0.0 };
	m_gripperGripForce = 0.0;

if (runningExperiment){
	// initialize experiment
	Exp.init();
	initRecording(Exp.m_subjNum);
	exp_clk.start(true);
	record_clk.setTimeoutPeriodSeconds(.001);
	record_clk.start(true);
}
	return errors;				// TO DO...... 
}

// initialization sub-functions
int hapticsThread::initializeChai3dStuff(void) {
	
	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

		// initialize GLFW library
		if (!glfwInit())
		{
			cout << "failed initialization" << endl;
			cSleepMs(1000);
			return 1;
		}


		// set error callback
		glfwSetErrorCallback(errorCallback);

		// compute desired size of window
		const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		int w = 0.8 * mode->height;
		int h = 0.5 * mode->height;
		int x = 0.5 * (mode->width - w);
		int y = 0.5 * (mode->height - h);



		// set OpenGL version
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

		if (stereoMode == C_STEREO_ACTIVE)								// set active stereo mode
		{
			glfwWindowHint(GLFW_STEREO, GL_TRUE);
		}
		else
		{
			glfwWindowHint(GLFW_STEREO, GL_FALSE);
		}

		window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);			// create display context

		if (!window)
		{
			cout << "failed to create window" << endl;
			cSleepMs(1000);
			glfwTerminate();
			return 1;
		}

		glfwGetWindowSize(window, &width, &height);						// get width and height of window
		glfwSetWindowPos(window, x, y);									// set position of window
		glfwMakeContextCurrent(window);									// set current display context
		glfwSwapInterval(swapInterval);									// sets the swap interval for the current display context




#ifdef GLEW_VERSION
		// initialize GLEW library
		if (glewInit() != GLEW_OK)
		{
			cout << "failed to initialize GLEW library" << endl;
			glfwTerminate();
			return 1;
		}
#endif


		//hapticsThread::setUpWorld();			// world - camera - lighting

		return 0;
}

int hapticsThread::setUpWorld(void) {

	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	world = new cWorld();					// create a new world.
	world->m_backgroundColor.setWhite();	// set the background color of the environment
	camera = new cCamera(world);			// create a camera and insert it into the virtual world
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(2.0, 0.0, 0.2),   // camera position (eye)
		cVector3d(0.0, 0.0, -0.5),			// lookat position (target)
		cVector3d(0.0, 0.0, 1.0));			// direction of the "up" vector

											// set the near and far clipping planes of the camera
											// anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);
	camera->setStereoMode(stereoMode);				// set stereo mode
	camera->setStereoEyeSeparation(0.02);			// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoFocalLength(2.0);
	camera->setMirrorVertical(mirroredDisplay);		// set vertical mirrored display mode

	light = new cSpotLight(world);			// create a light source
	camera->addChild(light);				// attach light to camera
	light->setEnabled(true);				// enable light source
	light->setLocalPos(0.0, 0.0, 3.0);		// position the light source
	light->setDir(0.0, 0.0, -1.0);			// define the direction of the light beam
	light->setSpotExponent(0.0);			// set uniform concentration level of light 
	light->setShadowMapEnabled(true);		// enable this light source to generate shadows
	light->m_shadowMap->setQualityLow();			// set the resolution of the shadow map
	//light->m_shadowMap->setQualityMedium();
	light->setCutOffAngleDeg(45);			// set light cone half angle

	return 0;
}

int hapticsThread::setUpHapticDevice(void) {


	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	//handler = new cHapticDeviceHandler();			// create a haptic device handler
	//handler->getDevice(hapticDevice, 0);				// get access to the first available haptic device
	//cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();		// retrieve information about the current haptic device
	//hapticDevice->setEnableGripperUserSwitch(true);			// if the haptic devices carries a gripper, enable it to behave like a user switch
	chaiMagDevice->setEnableGripperUserSwitch(true);
	tool = new cToolCursor(world);					// create a 3D tool and add it to the world
	world->addChild(tool);
	
	tool->setHapticDevice(chaiMagDevice);		// connect the haptic device to the tool
	toolRadius = 0.05;							// define the radius of the tool (sphere)
	tool->setRadius(toolRadius);				// define a radius for the tool
	tool->setShowContactPoints(false, false);		// hide the device sphere. only show proxy.
	tool->setShowFrame(false);
	tool->setFrameSize(0.5);
	tool->setWorkspaceRadius(1.0);		// map the physical workspace of the haptic device to a larger virtual workspace.



	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->enableDynamicObjects(false);


	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// initialize tool by connecting to haptic device
	tool->start();


	//tool->m_hapticPointFinger->m_sphereProxy->addChild(m_curSphere0);
	//tool->m_hapticPointThumb->m_sphereProxy->addChild(m_curSphere1);
	tool->setShowEnabled(true, true);
	//loadFingerMeshes();
	return 0;
}


void hapticsThread::setUpWidgets(void) {				// ----------------TO DO --------------------------------
	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	font = NEW_CFONTCALIBRI20();		//create a font

	// create a label to display the haptic and graphic rate of the simulation
	labelTrial = new cLabel(font);
	labelTrial->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelTrial);

	labelStatus= new cLabel(font);
	labelStatus->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelStatus);

}

void hapticsThread::loadFingerMeshes(void) {
	
	//--------------------------------------------------------------------------
	// FINGER MESHES
	//--------------------------------------------------------------------------
	finger = new chai3d::cMultiMesh(); // create the finger
	world->addChild(finger);	
	finger->setFrameSize(0.5);			
	finger->setLocalPos(0.0, 0.0, 0.0);

	thumb = new chai3d::cMultiMesh(); //create the thumb
	world->addChild(thumb);
	thumb->setFrameSize(0.5);
	thumb->setLocalPos(0, 0, 0);

	// load an object file
	if (cLoadFileOBJ(finger, "../Resources/FingerModel.obj")) {
		cout << "Finger file loaded." << endl;
	}
	else {
		cout << "Failed to load finger model." << endl;
	}
	if (cLoadFileOBJ(thumb, "../Resources/FingerModelThumb.obj")) {
		cout << "Thumb file loaded." << endl;
	}

	// set params for finger
	finger->scale(7);
	finger->setShowEnabled(false);
	finger->setUseVertexColors(true);
	chai3d::cColorf fingerColor;
	fingerColor.setBrownSandy();
	finger->setVertexColor(fingerColor);
	finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
	finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	finger->m_material->m_specular.set(1.0, 1.0, 1.0);
	finger->setUseMaterial(true);
	finger->setHapticEnabled(false);
	finger->setShowFrame(false);

	// set params for thumb
	thumb->scale(7);
	thumb->setShowEnabled(false);
	thumb->setUseVertexColors(true);
	chai3d::cColorf thumbColor;
	thumbColor.setBrownSandy();
	thumb->setVertexColor(thumbColor);
	thumb->m_material->m_ambient.set(0.1, 0.1, 0.1);
	thumb->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	thumb->m_material->m_specular.set(1.0, 1.0, 1.0);
	thumb->setUseMaterial(true);
	thumb->setHapticEnabled(false);
	thumb->setShowFrame(false);


}


// main thread loop
void hapticsThread::updateHaptics(void)     
{
	// angular velocity
	cVector3d angVel(0, 0, 0);

	// reset clock
	cPrecisionClock clock;
	clock.reset();

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////
		// SIMULATION TIME
		/////////////////////////////////////////////////////////////////////

		clock.stop();				// stop the simulation clock
		double timeInterval = clock.getCurrentTimeSeconds(); 		// read the time increment in seconds
		clock.reset();			// restart the simulation clock
		clock.start();
		hapticRate.signal(1);			// signal frequency counter

		/////////////////////////////////////////////////////////////////////
		// TOOL POSITION UPDATE
		/////////////////////////////////////////////////////////////////////

		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		// update position and orientation of tool
		tool->updateFromDevice();				// ----- TO DO : CHECK THAT THIS UPDATES PANTOGRAPH ANGLES
		
		m_worldLock.acquire();
		//m_position = tool->m_hapticPoint->getGlobalPosGoal();
		chaiMagDevice->getPosition(m_position);
		chaiMagDevice->getRotation(m_rotation);
		m_worldLock.release();


		/////////////////////////////////////////////////////////////////////
		// GET FORCES FOR HAPTIC GRIPPER
		/////////////////////////////////////////////////////////////////////
		if (runningExperiment) {
			
			m_gripperTorque.set(0.0, 0.0, 0.0);
			m_gripperGripForce = 0.0;
			m_gripperForce.set(0.0, 0.0, 0.0);

			if (Exp.m_trial <= MAX_TRIALS) {
				
				switch(Exp.state){


				case experiment::startingCue :
				// Get cue information for this trial
					Exp.getCue(m_cue, m_magnitude);
				// reset clock
					exp_clk.reset();
					exp_t = exp_clk.getCurrentTimeSeconds();
					exp_t_start = exp_clk.getCurrentTimeSeconds();
					cout << "Starting Trial " << Exp.m_trial<< endl;

					Exp.state = experiment::applyingCue;

				break;

				case experiment::applyingCue :
				// Apply cue
					if( (exp_t - exp_t_start) < m_cueDuration) {	
						exp_t = exp_clk.getCurrentTimeSeconds();
						Exp.getCueForce(m_cue, m_magnitude, exp_t, exp_t_start, m_fingerForce, m_thumbForce, m_cueDuration);
						m_runLock.acquire();
						m_gripper->setForcesAndTorques(m_gripperForce, m_gripperTorque, m_gripperGripForce, m_thumbForce, m_fingerForce);
						m_gripper->motorLoop();					// Output to motors
						m_runLock.release();
					}
					else{
						Exp.state = experiment::startingReturn;
						Sleep(1000);
					}
				break;
				
				case experiment::startingReturn :

					// Return slowly to center postion
					thumbForce_start = m_thumbForce; 
					fingerForce_start = m_fingerForce;
					exp_t_start = exp_clk.getCurrentTimeSeconds();
					exp_t = exp_clk.getCurrentTimeSeconds();
					Exp.state = experiment::returningToCenter;
					break;

				case experiment::returningToCenter :
					if( (exp_t - exp_t_start) < m_returnDuration) {		
						exp_t = exp_clk.getCurrentTimeSeconds();
						Exp.getReturnForce(exp_t, exp_t_start, fingerForce_start, thumbForce_start, m_fingerForce, m_thumbForce, m_returnDuration);
						// send to gripper object
						m_runLock.acquire();
						m_gripper->setForcesAndTorques(m_gripperForce, m_gripperTorque, m_gripperGripForce, m_thumbForce, m_fingerForce);
						m_gripper->motorLoop();						// Output to motors
						m_runLock.release();
					}
					else{
						Exp.state = experiment::startingCue;
						Exp.m_trial++;		// increment trial number
					}
			
				}	// switch case

				// record position information at 100 Hz
				if (record_clk.timeoutOccurred()) {
					record();
					record_clk.setTimeoutPeriodSeconds(0.001);
					record_clk.start(true);
				}


			}	// if trials still left to go

		}	// if experiment
		//updateGraphics();

	}		// simulation running loop
		

	// exit haptics thread
	m_runLock.acquire();
	m_gripper->disableCtrl();	// set motors to zero volts
	simulationFinished = true;
	m_gripper->disconnect();
	m_runLock.release();
}


//------------------------------------------------------------------------------

void hapticsThread::updateGraphics(void)
{	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update trial number
	if (Exp.m_trial <= MAX_TRIALS) {
		labelTrial->setText("Trial: " + to_string(Exp.m_trial));
	}

	// update status
	if (Exp.state == experiment::returningToCenter) {
		labelStatus->setText(" ");
	}
	else if (Exp.state == experiment::startingCue || Exp.state == experiment::applyingCue) {
		labelStatus->setText("Giving Cue: " + Exp.cue_type[m_cue]);
	}

	// update position of labels
	labelTrial->setLocalPos((int)(0.5 * (width - labelTrial->getWidth())), 0.5*height);
	labelStatus->setLocalPos((int)(0.5 * (width - labelTrial->getWidth())), 0.4*height);


	/////////////////////////////////////////////////////////////////////
	// UPDATE WORLD
	/////////////////////////////////////////////////////////////////////

	world->updateShadowMaps(false, mirroredDisplay); 	// update shadow maps (if any)
	camera->renderView(width, height);          	// render world	     
	glFinish();		// wait until all GL commands are completed


	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}


//------------------------------------------------------------------------------
// Helpers
//------------------------------------------------------------------------------

bool hapticsThread::checkSimulationStatus(void) {
	if (simulationFinished) {
		return true;
	}
	else {
		return false;	}
}


void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}



void hapticsThread::initRecording(int s) {
	// set up file to record 
	 file.open("../UserData_" + std::to_string(s) + ".txt");
}

void hapticsThread::record(void){			

		/* 
		trial
		cue
		time
		position xyz
		rotation matrix
		xy index position
		xy thumb position
		*/

		file << Exp.m_trial << ", " << m_cue << ", " << m_magnitude << ", " << exp_t << ", ";
		file << m_position.x() << ", " << m_position.y() << ", " << m_position.z() << ", ";
		file << m_rotation.getRow(0) << ", " << m_rotation.getRow(1) << ", " << m_rotation.getRow(2) << ", ";
		file << m_gripper->m_thDes[0] << ", " << m_gripper->m_thDes[1] << ", " << m_gripper->m_thDes[2] << ", " << m_gripper->m_thDes[3] << ", ";
		file << m_gripper->m_th[0] << ", " << m_gripper->m_th[1] << ", " << m_gripper->m_th[2] << ", " << m_gripper->m_th[3];
		file << endl;



}