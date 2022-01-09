#ifndef KortexRobot_h
#define KortexRobot_h

#include <KDetailedException.h>
#include <KError.h>
#include <SessionManager.h>
#include <SessionClientRpc.h>
#include <ControlConfigClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <InterconnectConfigClientRpc.h>
#include <google/protobuf/util/json_util.h>
#include "utilities.h"

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#define FB_TYPE_NONE 0
#define FB_TYPE_JOINT_P 1
#define FB_TYPE_JOINT_V 2
#define FB_TYPE_JOINT_T 3
#define FB_TYPE_TOOL_P 4
#define FB_TYPE_TOOL_V 5
#define FB_TYPE_TOOL_FT 6
#define FB_TYPE_COLLISIONS 7

struct tCartesianVector
{
	float x,y,z;
	tCartesianVector(const float& X, const float& Y, const float& Z): x(X), y(Y), z(Z) {};
	tCartesianVector operator+(const tCartesianVector v) const {return tCartesianVector(x + v.x, y + v.y, z + v.z);}
	float Norm() const {return sqrtf(x*x + y*y + z*z);}
	tCartesianVector Normalized() const
	{
		float norm = Norm();
		if (norm > 0.0f)
			return tCartesianVector(x/norm, y/norm, z/norm);
		else 
			return tCartesianVector(0.0f, 0.0f, 0.0f); 
		
	}
};

class KortexRobot {
public: 
	KortexRobot(const std::string& IP);
	~KortexRobot(){{Disconnect();}};

	bool Init();
	void Disconnect(); 
	bool IsConnected() {return m_bIsConnected;}
	
	
	// Feedback
	bool RefreshFeedback();
	Kinova::Api::BaseCyclic::Feedback GetFeedback() const {return m_Feedback;}
	Kinova::Api::BaseCyclic::ActuatorFeedback GetActuatorFeedback(const int actIdx) const;
	Kinova::Api::Base::Pose GetCartesianPose() const;
	Kinova::Api::Base::Twist GetCartesianTwist() const;
	Kinova::Api::Base::Wrench GetCartesianWrench() const;
	std::vector<float> GetJointPositions() const;
	std::vector<float> GetJointVelocities() const;
	std::vector<float> GetJointTorques() const;

	// Check data
	bool SetCustomData(std::vector<float> data);
	
	// Actions
	bool ExecuteExistingAction(const std::string& actionName, Kinova::Api::Base::RequestedActionType& actionType);
	bool MoveTo(const tCartesianVector& position, const tCartesianVector& orientation, const Kinova::Api::Base::CartesianTrajectoryConstraint& constraint);
	bool SendTwistCommand(const tCartesianVector& translation, const tCartesianVector& rotation);
	bool SetJointAngles(const std::vector<float>& angles, const Kinova::Api::Base::JointTrajectoryConstraint& constraints);
	bool SetJointSpeeds(std::vector<float>& jointSpeeds);
	bool PlaySequence(const Kinova::Api::Base::SequenceHandle& sequenceHandle);
	bool Stop();
	bool MoveGripper(float vel);
	bool forKin();
	bool invKin();
	bool SetupLowLevelControl();
	bool LowLevelMove(std::vector<float> jointpos);
	
	// helper functions
	int64_t GetTickUs();
	int GetNbDoF() const {return m_nNbDoF;}
	bool SetTwistReferenceFrame(const  Kinova::Api::Common::CartesianReferenceFrame& frame);
	bool ExecuteAction(const Kinova::Api::Base::Action& action);
	bool WaitWhileRobotIsMoving(const int timeout);
	bool WaitWhileRobotIsMoving_Twist(const int timeout);
	Kinova::Api::Base::SequenceHandle CreateSequence(const Kinova::Api::Base::Sequence& sequence) {return m_pBase->CreateSequence(sequence);}
	
	// notifications
	void SubscribeToNotifications();
	void UnsubscribeToNotifications();
	void PrintFeedback(int feedbackType);

protected:
	void OnError(Kinova::Api::KDetailedException& ex);
	void OnActionNotificationCallback(Kinova::Api::Base::ActionNotification notif);
	void OnRefreshFeedbackCallback(const Kinova::Api::Error& notif, const Kinova::Api::BaseCyclic::Feedback& feedback) {m_Feedback = feedback;}

protected: 
	int m_nNbDoF;
	bool m_bIsBusy;
	std::string m_sIP;
	bool m_bIsConnected;
	std::vector<Kinova::Api::Common::NotificationHandle> m_NotificationHandleList;
	Kinova::Api::ControlConfig::ControlConfigClient* m_pControlConfigClient;
	Kinova::Api::DeviceConfig::DeviceConfigClient* m_pDeviceConfigClient;
	Kinova::Api::SessionManager* m_pSessionManager;
	Kinova::Api::TransportClientTcp* m_pTcpClient;
	Kinova::Api::RouterClient* m_pRouterClient;
	Kinova::Api::Base::BaseClient* m_pBase;
	Kinova::Api::Base::Action m_Action;

	Kinova::Api::TransportClientUdp* m_pUdpClient;
	Kinova::Api::RouterClient* m_pRouterClientRT;
	Kinova::Api::SessionManager* m_pSessionManagerRT;
	Kinova::Api::BaseCyclic::BaseCyclicClient* m_pBaseCyclic;
	Kinova::Api::BaseCyclic::Feedback m_Feedback;
	 
	
	std::vector<float> m_CustomData;
	Kinova::Api::BaseCyclic::Command  m_BaseCyclicCommand;
    std::vector<float> m_Commands;
};

#endif /* KortexRobot_h */