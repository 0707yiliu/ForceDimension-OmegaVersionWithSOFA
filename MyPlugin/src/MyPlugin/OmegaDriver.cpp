#include <MyPlugin/OmegaDriver.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/ScriptEvent.h>
#include <sofa/simulation/Node.h>

#include <sofa/core/visual/VisualParams.h>
#include <thread>
#include <chrono>

#include <cmath>


namespace sofa::component::controller
{
using namespace sofa::defaulttype;

OmegaDriver::OmegaDriver()
	: d_deviceName(initData(&d_deviceName, std::string("Omega.7"), "deviceName", "Name of device Configuration"))
	, d_positionBase(initData(&d_positionBase, Vec3d(0,0,0), "positionBase","Position of the device base in the SOFA scene world coordinates"))
	, d_orientationBase(initData(&d_orientationBase, Quat(0,0,0,1), "orientationBase","Orientation of the device base in the SOFA scene world coordinates"))
	, d_orientationTool(initData(&d_orientationTool, Quat(0,0,0,1), "orientationTool","Orientation of the tool in the SOFA scene world coordinates"))
	, d_scale(initData(&d_scale, 1.0, "scale","Default scale applied to the Device coordinates"))
	, d_forceScale(initData(&d_forceScale, 1.0, "forceScale", "Default forceScale applied to the force feedback"))
	, d_maxInputForceFeedback(initData(&d_maxInputForceFeedback, double(1.0), "maxInputForceFeedback", "Maximum value of the normed input force feedback for device security"))
	, d_inputForceFeedback(initData(&d_inputForceFeedback, Vec3d(0, 0, 0), "inputForceFeedback", "Input force feedback in case of no LCPForceFeedback is found (manual setting)"))
	, d_manualStart(initData(&d_manualStart, false, "manualStart", "If true, will not automatically initDevice at component init phase."))
        , d_emitButtonEvent(initData(&d_emitButtonEvent, false, "emitButtonEvent", "If true, will send event through the graph when button are pushed/released"))
        , d_frameVisu(initData(&d_frameVisu, false, "drawDeviceFrame", "Visualize the frame corresponding to the device tooltip"))
        , d_omegaVisu(initData(&d_omegaVisu, false, "drawDevice", "Visualize the Geomagic device in the virtual scene"))
        , d_enableForce(initData(&d_enableForce, true, "enableForce", "Omega Device ForceFeedback enable flag"))
	, d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))
	, d_angle(initData(&d_angle, "angle", "Angluar values of joint (rad)"))
	, d_button_1(initData(&d_button_1,"button1","Button state 1"))
//	, d_omegadx(initData(&d_omegadx, "omega_dx","Position of the device in X-axe"))
//        , d_omegady(initData(&d_omegadt, "omega_dy","Position of the device in Y-axe"))
//        , d_omegadz(initData(&d_omegadz, "omega_dz","Position of the device in Z-axe"))
	, l_forceFeedback(initLink("forceFeedBack", "link to the forceFeedBack component, if not set will search through graph and take first one encountered."))
	, m_simulationStarted(false)
	, m_isInContact(false)
	, m_DeviceID(0)
{
	this->f_listening.setValue(true);
	m_forceFeedback = nullptr;
	sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Loading);
}

OmegaDriver::~OmegaDriver()
{
	clearDevice();
}

void OmegaDriver::init()
{
	if (l_forceFeedback.empty())
	{
	        sofa::simulation::Node *context = dynamic_cast<sofa::simulation::Node *>(this->getContext());
		m_forceFeedback = context->get<ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
	}
	else
	{
		m_forceFeedback = l_forceFeedback.get();
	}

	if (!m_forceFeedback.get())
	{
		msg_warning() << "No forceFeedBack component found in the scene. Only the motion of the haptic tool will be simulated.";
	}

	if(d_manualStart.getValue() == false)
	{
		initDevice();
	}


}
void OmegaDriver::initDevice()
{
	//init device
	int omega_major, omega_minor, omega_release, omega_revision;
	dhdGetSDKVersion(&omega_major, &omega_minor, &omega_release, &omega_revision);
	printf ("\n");
	printf ("Force Dimension - Gravity Compensation Example %d.%d.%d.%d\n", omega_major, omega_minor, omega_release, omega_revision);
	printf ("Copyright (C) 2001-2020 Force Dimension\n");
 	printf ("All Rights Reserved.\n\n");
	// open the first available device
  	if (dhdOpen () < 0) {
    		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    		dhdSleep (2.0);
    	return;
  	}
 	// identify device
  	printf ("%s device detected\n\n", dhdGetSystemName());
	// display instructions
  	printf ("If you want to quit, please FOLLOW the SOFA GUI regulation and shutdown the force button manually\n\n");
	// enable force
  	dhdEnableForce (DHD_ON);

	//Set max ForceFeedback security
	//TODO max force should be used in processing
	if (d_maxInputForceFeedback.isSet())
	{
		msg_info() << "maxInputForceFeedback value (" << d_maxInputForceFeedback.getValue() << ") is set, carefully set the max force regarding your haptic device";
		if (d_maxInputForceFeedback.getValue() <= 0.0)
		{
			msg_error() << "maxInputForceFeedback value (" << d_maxInputForceFeedback.getValue() << ") is negative or 0, it should be strictly positive";
            		d_maxInputForceFeedback.setValue(0.0);
		}
	}
	
	//Need to wait several ms for the scheduler to be well launched and retrieving correct device information before updating information on the SOFA side
	std::this_thread::sleep_for(std::chrono::milliseconds(42));
	updatePosition();
// 	std::thread thread1(updatePosition);
//	thread1.join();
	sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);

}
void OmegaDriver::clearDevice()
{
	dhdClose ();
	msg_info() << "done.";
}
void OmegaDriver::reinit()
{
}

void OmegaDriver::updatePosition()
{
	//SOFA_UNUSED(1.0);
	//
	//double & omegadx = *d_omegadx.beginEdit();
	//double & omegady = *d_omegady.beginEdit();
	//double & omegadz = *d_omegadz.beginEdit();
	double px,py,pz;
	double oa,ob,og;
//	double pos_scale = 10;
	double q_x = 0,q_y = 0,q_z = 0,q_w = 1;
//	Vector6 & angle = *d_angle.beginEdit();
	bool done = true;
//	while(done){
	OmegaDriver::Coord & posDevice = *d_posDevice.beginEdit();
//	Vector6 & posDevice = *d_posDevice.beginEdit();
	const Vector3 & positionBase = d_positionBase.getValue();
	const Quat & orientationBase = d_orientationBase.getValue();
    	const Quat & orientationTool = d_orientationTool.getValue();
	const double & scale = d_scale.getValue();
	const bool & enableForce = d_enableForce.getValue();
	updateButtonStates();
	if (dhdGetPosition (&px, &py, &pz) < DHD_NO_ERROR) {
        	printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        	done = false;
      //  	return;
      	}
	Vector3 position;
	double ex_x = 3, ex_y = 3, ex_z = 3;
	position[0] = -px;
	position[1] = pz;
	position[2] = py;
//	printf("get the position x: %f, y: %f, z: %f\n", px,py,pz);
	if (dhdGetOrientationRad(&oa, &ob, &og) < DHD_NO_ERROR) {
        	printf ("error: cannot read orientation (%s)\n", dhdErrorGetLastStr());
        	done = false;
      //  	return;
	}
//	printf("get the orientation oa: %f, ob: %f, og: %f\n", oa,ob,og);
	//TODO Have got EulerAngle, Next, we need to transform the EulerAngle to Quaternion
	
//	double cy = cos(og * 0.5);
//	double sy = sin(og * 0.5);
//	double cp = cos(ob * 0.5);
//	double sp = sin(ob * 0.5);
//	double cr = cos(oa * 0.5);
//	double sr = sin(oa * 0.5);
//
//	q_x = cy * cp * cr - sy * sp * sr;
//	q_y = sy * cp * sr + cy * sp * cr;
//	q_z = sy * cp * cr - cy * sp * sr;
//	q_x = cy * cp * cr + sy * sp * sr;

	//EulerAngleToTransformMatrix
	//TODO This method will make the TransformMatrix as middle value
//	float angle[3];
	int times = 3, row = 3, col = 3, num = 0;
	double Matrix[9];
	double Matrix_result[9];
//	angle[0] = oa;
//	angle[1] = ob;
//	angle[2] = og;
	double tempZ[9] = {cos(ob), -sin(ob), 0, sin(ob), cos(ob), 0, 0, 0, 1};
//	double tempZ[9] = {cos(0), -sin(0), 0, sin(0), cos(0), 0, 0, 0, 1};
	double tempY[9] = {cos(og), 0, sin(og), 0, 1, 0, -sin(og), 0, cos(og)};
	double tempX[9] = {1, 0, 0, 0, cos(oa), -sin(oa), 0, sin(oa), cos(oa)};
	for (int i = 1; i <= row; i++)
  	{
    		for (int j = 1; j <= col; j++)
    		{
      			Matrix[num] = 0;
      			for (int k = 0; k <= times - 1; k++)
      			{
        			Matrix[num] += tempZ[(i-1) * col + k] * tempY[k * col + j - 1];
      			}
      			num++;
    		}
  	}	
	int num_result = 0;
  	for (int i = 1; i <= row; i++)
  	{
    		for (int j = 1; j <= col; j++)
    		{
      			Matrix_result[num_result] = 0;
      			for (int k = 0; k <= times - 1; k++)
      			{
        			Matrix_result[num_result] += Matrix[(i-1) * col + k] * tempX[k * col + j - 1];
      			}
      			num_result++;
    		}
  	}


	//copy rotation of the tool
    	Mat3x3d mrot;
	num = 0;
    	for (int u=0; u<3; u++)
	{
        	for (int j=0; j<3; j++)
		{
           		mrot[u][j] = Matrix_result[num];
			num++;
		}
	}
	Quat orientation;
	orientation.fromMatrix(mrot);
//	orientation[0] = q_x;
//	orientation[1] = q_y;
//	orientation[2] = q_z;
//	orientation[3] = q_w;
	//compute the position of the tool (according to positionbase, orientation base and the scale
	posDevice.getCenter() = positionBase + orientationBase.rotate(position*scale);
	posDevice.getOrientation() = orientationBase * orientation * orientationTool;

        //****************Force Feedback Porcessing***************** 
	Vector3 currentForce;
        if (m_forceFeedback)
        {
		Vector3 pos_in_world = positionBase + orientationBase.rotate(position*scale);
		m_forceFeedback->computeForce(pos_in_world[0],pos_in_world[1],pos_in_world[2], 0, 0, 0, 0, currentForce[0], currentForce[1], currentForce[2]);

        }
//	double fx = 3, fy = 3, fz = 3;
        if (enableForce)
        {

	printf("ForceFeedback value : %f, %f, %f\n", currentForce[0], currentForce[1], currentForce[2]);
//	printf("ForceFeedback value : %f\n", currentForce[0]);
                // retrieve force
/*                if (dhdGetForce (&currentForce[0], &currentForce[1], &currentForce[2]) < DHD_NO_ERROR) {
                        printf ("error: cannot read force (%s)\n", dhdErrorGetLastStr());
                        done = false;
                }*/
                /*if (dhdGetForce (&fx, &fy, &fz) < DHD_NO_ERROR) {
                        printf ("error: cannot read force (%s)\n", dhdErrorGetLastStr());
                        done = false;
                }*/
		if (dhdSetForceAndTorqueAndGripperForce (currentForce[0], currentForce[1], currentForce[2], 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR){
			printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			done = false;
		}

        }
        //**********************************************************

//	printf("ForceFeedback value : %f\n", currentForce[0]);
//	posDevice[0] = position[0];
//	posDevice[1] = position[1];
//	posDevice[2] = position[2];
//	posDevice[3] = 1;
//	posDevice[4] = 1;
//	posDevice[5] = 1;
	d_posDevice.endEdit();

//	}

}
void OmegaDriver::updateButtonStates()
{
	SOFA_UNUSED(1.0);
}

void OmegaDriver::handleEvent(core::objectmodel::Event *event)
{
	if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
	{
		if (sofa::core::objectmodel::BaseObject::d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
			return;
		m_simulationStarted = true;
		updatePosition();
	}
}
int OmegaDriverClass = core::RegisterObject("Driver allowing interfacing with Omega Haptic Device.").add< OmegaDriver >();

}// namespace sofa::component::omegacontroller
