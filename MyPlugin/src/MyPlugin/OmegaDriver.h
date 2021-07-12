#pragma once

#include <MyPlugin/config.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/Quater.h>
#include <SofaUserInteraction/Controller.h>

#include <SofaHaptics/ForceFeedback.h>

#include "dhdc.h"
#include "drdc.h"

namespace sofa::component::controller
{
using namespace sofa::defaulttype;

class SOFA_MYPLUGIN_API OmegaDriver : public Controller
{

	public:
		SOFA_CLASS(OmegaDriver, Controller);
		typedef RigidTypes::Coord Coord;
		typedef RigidTypes::VecCoord VecCoord;


	protected:
		OmegaDriver();
		virtual ~OmegaDriver();
	public:
		void init() override;
		void handleEvent(core::objectmodel::Event *) override;
		virtual void initDevice();
		virtual void clearDevice();
		void reinit() override;
	protected:
		void updatePosition();
		void updateButtonStates();
	public:
		//Input Data
		Data< std::string > d_deviceName;
		Data<Vec3d> d_positionBase;///< Input Position of the device base in the scene world coordinates
		Data<Quat> d_orientationBase; ///< Input Orientation of the device base in the scene world coordinates
		Data<Quat> d_orientationTool; ///< Input Orientation of the tool
		Data<SReal> d_scale; ///< Default scale applied to the device Coordinates
		Data<SReal> d_forceScale; ///< Default forceScale applied to the force feedback.
		Data<SReal> d_maxInputForceFeedback; ///< Maximum value of the normed input force feedback for device security
		Data<Vector3> d_inputForceFeedback; ///< Input force feedback in case of no LCPForceFeedback is found (manual setting)
		//Input Parameters
		Data<bool> d_manualStart; /// < Bool to unactive the automatic start of the device at init. initDevice need to be called manually. False by default.
		Data<bool> d_emitButtonEvent; ///< Bool to send event through the graph when button are pushed/released
		Data<bool> d_frameVisu; ///< Visualize the frame corresponding to the device tooltip
		Data<bool> d_omegaVisu; ///< Visualize the frame of the interface in the virtual scene
		Data<bool> d_enableForce; /// the forceFeedback enable flag

		//Output Data
		Data<Coord> d_posDevice; ///< position of the base of the part of the device
		Data<Vector6> d_angle; ///< Angluar values of joint (rad)
		Data<bool> d_button_1; ///< Button state 1, maybe have much button in the future
	//	Data<double> d_omegadx; //OmegaDevice EE Position
	//        Data<double> d_omegady;
	//	Data<double> d_omegadz;	

		ForceFeedback::SPtr m_forceFeedback;
		SingleLink<OmegaDriver, ForceFeedback, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_forceFeedback;
		struct DeviceData
		{
			double eulerAngle[3];
			double transform[16];
			int buttonState;
		};
		bool m_simulationStarted;
		bool m_isInContact;
		DeviceData m_hapticData;
		DeviceData m_simuData;
		int m_DeviceID;
		std::vector< unsigned long > m_hStateHandles;
		//Data<unsigned> d_omegatrans;
		//Data<unsigned> d_omegarots;
};


}
