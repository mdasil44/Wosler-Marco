#include "Classes/include/KortexRobot.hpp"

KortexRobot::KortexRobot(const std::string &IP)
{
	m_sIP = IP;
	m_bIsConnected = false;
	m_bIsBusy = false;
	Init();
}

void KortexRobot::Disconnect()
{
	if (m_bIsConnected)
	{
		m_pSessionManager->CloseSession();
		m_pSessionManagerRT->CloseSession();
		m_pRouterClient->SetActivationStatus(false);
		m_pRouterClientRT->SetActivationStatus(false);
		m_pTcpClient->disconnect();
		m_pUdpClient->disconnect();
	}

	m_bIsConnected = false;
	if (m_pControlConfigClient != nullptr)
	{
		delete m_pControlConfigClient;
		m_pControlConfigClient = nullptr;
	}
	if (m_pDeviceConfigClient != nullptr)
	{
		delete m_pDeviceConfigClient;
		m_pDeviceConfigClient = nullptr;
	}
	if (m_pSessionManager != nullptr)
	{
		delete m_pSessionManager;
		m_pSessionManager = nullptr;
	}
	if (m_pRouterClient != nullptr)
	{
		delete m_pRouterClient;
		m_pRouterClient = nullptr;
	}
	if (m_pTcpClient != nullptr)
	{
		delete m_pTcpClient;
		m_pTcpClient = nullptr;
	}
	if (m_pSessionManagerRT != nullptr)
	{
		delete m_pSessionManagerRT;
		m_pSessionManagerRT = nullptr;
	}
	if (m_pRouterClientRT != nullptr)
	{
		delete m_pRouterClientRT;
		m_pRouterClientRT = nullptr;
	}
	if (m_pUdpClient != nullptr)
	{
		delete m_pUdpClient;
		m_pUdpClient = nullptr;
	}
	if (m_pBaseCyclic != nullptr)
	{
		delete m_pBaseCyclic;
		m_pBaseCyclic = nullptr;
	}
	//	Autoformat messses it up replace with this
	// if (m_pControlConfigClient != nullptr){delete m_pControlConfigClient; m_pControlConfigClient = nullptr;}
	// if (m_pDeviceConfigClient != nullptr){delete m_pDeviceConfigClient; m_pDeviceConfigClient = nullptr;}
	// if (m_pSessionManager != nullptr){delete m_pSessionManager; m_pSessionManager = nullptr;}
	// if (m_pRouterClient != nullptr){delete m_pRouterClient; m_pRouterClient = nullptr;}
	// if (m_pTcpClient != nullptr){delete m_pTcpClient; m_pTcpClient = nullptr;}
	// if (m_pSessionManagerRT != nullptr){delete m_pSessionManagerRT; m_pSessionManagerRT = nullptr;}
	// if (m_pRouterClientRT != nullptr){delete m_pRouterClientRT; m_pRouterClientRT = nullptr;}
	// if (m_pUdpClient != nullptr){delete m_pUdpClient; m_pUdpClient = nullptr;}
	// if (m_pBaseCyclic != nullptr){delete m_pBaseCyclic; m_pBaseCyclic = nullptr;}
}

bool KortexRobot::Init()
{
	if (m_bIsConnected)
		Disconnect();

	// Create API objects
	auto error_callback = [](Kinova::Api::KError err)
	{ cout << "callback error" << err.toString(); };
	m_pTcpClient = new Kinova::Api::TransportClientTcp();
	m_pRouterClient = new Kinova::Api::RouterClient(m_pTcpClient, error_callback);
	auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
	create_session_info.set_username("admin");
	create_session_info.set_password("admin");
	create_session_info.set_session_inactivity_timeout(60000);	 // (milliseconds)
	create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

	if (!m_pTcpClient->connect(m_sIP, 10000))
	{
		std::cout << "Failed to Connect" << std::endl;
		Disconnect();
		return false;
	}

	try
	{
		m_pSessionManager = new Kinova::Api::SessionManager(m_pRouterClient);
		m_pSessionManager->CreateSession(create_session_info);

		m_pDeviceConfigClient = new Kinova::Api::DeviceConfig::DeviceConfigClient(m_pRouterClient);
		m_pBase = new Kinova::Api::Base::BaseClient(m_pRouterClient);
		m_pControlConfigClient = new Kinova::Api::ControlConfig::ControlConfigClient(m_pRouterClient);

		m_nNbDoF = m_pBase->GetActuatorCount().count();

		auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
		servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING); // SINGLE_LEVEL_SERVOING
		m_pBase->SetServoingMode(servoingMode);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
		Disconnect();
		return false;
	}

	m_pUdpClient = new Kinova::Api::TransportClientUdp();
	m_pRouterClientRT = new Kinova::Api::RouterClient(m_pUdpClient, error_callback);

	if (!m_pUdpClient->connect(m_sIP, 10001))
	{
		std::cout << "Failed to connect cyclic" << std::endl;
		Disconnect();
		return false;
	}

	try
	{
		m_pSessionManagerRT = new Kinova::Api::SessionManager(m_pRouterClientRT);
		m_pSessionManagerRT->CreateSession(create_session_info);
		m_pBaseCyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(m_pRouterClientRT);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
		Disconnect();
		return false;
	}
	std::cout << "Connection Successful" << std::endl;
	m_bIsConnected = true;
	return true;
}

void KortexRobot::SubscribeToNotifications()
{
	if (!m_bIsConnected)
		return;

	Kinova::Api::Common::NotificationOptions options;
	options.set_type(Kinova::Api::Common::NOTIFICATION_TYPE_EVENT);

	using namespace std::placeholders;

	std::function<void(Kinova::Api::Base::ActionNotification)> actionCallback = std::bind(&KortexRobot::OnActionNotificationCallback, this, _1);
	auto handle = m_pBase->OnNotificationActionTopic(actionCallback, options);
	m_NotificationHandleList.push_back(handle);
}

void KortexRobot::UnsubscribeToNotifications()
{
	if (!m_bIsConnected)
		return;

	for (auto handle : m_NotificationHandleList)
	{
		m_pBase->Unsubscribe(handle);
	}
}

void KortexRobot::OnError(Kinova::Api::KDetailedException &ex)
{
	auto error_info = ex.getErrorInfo().getError();
	std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

	std::cout << "KError error_code: " << error_info.error_code() << std::endl;
	std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
	std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

	// Error codes by themselves are not very verbose if you don't see their corresponding enum value
	// You can use google::protobuf helpers to get the string enum element for every error code and sub-code
	std::cout << "Error code string equivalent: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(error_info.error_code())) << std::endl;
	std::cout << "Error sub-code string equivalent: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
}

void KortexRobot::OnActionNotificationCallback(Kinova::Api::Base::ActionNotification notif)
{
	switch (notif.action_event())
	{
	case Kinova::Api::Base::ACTION_START:
	{
		m_bIsBusy = true;
		// std::cout << "Action ID " << notif.handle().identifier() << " has started" << std::endl;
		break;
	}
	case Kinova::Api::Base::ACTION_END:
	{
		m_bIsBusy = false;
		// std::cout << "Action ID " << notif.handle().identifier() << " has ended" << std::endl;
		break;
	}
	case Kinova::Api::Base::ACTION_ABORT:
	{
		m_bIsBusy = false;
		// std::cout << "Action ID " << notif.handle().identifier() << " has aborted" << std::endl;
		break;
	}
	case Kinova::Api::Base::ACTION_PAUSE:
	{
		m_bIsBusy = false;
		// std::cout << "Action ID " << notif.handle().identifier() << " was paused" << std::endl;
		break;
	}
	default:
	{
		// we dont care about the other the other notifications.
	}
	}
}

bool KortexRobot::ExecuteExistingAction(const std::string &actionName, Kinova::Api::Base::RequestedActionType &actionType)
{
	if (!m_bIsConnected || m_bIsBusy)
		return false;

	try
	{
		auto action_list = m_pBase->ReadAllActions(actionType);
		auto action_handle = Kinova::Api::Base::ActionHandle();
		action_handle.set_identifier(0);
		for (auto action : action_list.action_list())
		{
			if (action.name() == actionName)
				action_handle = action.handle();
		}
		if (action_handle.identifier() == 0)
			return false;
		else
			m_pBase->ExecuteActionFromReference(action_handle);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return true;
}

bool KortexRobot::MoveTo(const tCartesianVector &position, const tCartesianVector &orientation, const Kinova::Api::Base::CartesianTrajectoryConstraint &constraint)
{
	if (!m_bIsConnected || m_bIsBusy)
		return false;

	m_Action.mutable_handle()->set_action_type(Kinova::Api::Base::REACH_POSE);
	m_Action.mutable_reach_pose()->mutable_target_pose()->set_x(position.x);
	m_Action.mutable_reach_pose()->mutable_target_pose()->set_y(position.y);
	m_Action.mutable_reach_pose()->mutable_target_pose()->set_z(position.z);
	m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_x(orientation.x);
	m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_y(orientation.y);
	m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_z(orientation.z);

	switch (constraint.type_case())
	{
	case Kinova::Api::Base::CartesianTrajectoryConstraint::TypeCase::kDuration:
	{
		m_Action.mutable_reach_pose()->mutable_constraint()->set_duration(constraint.duration());
	}
	case Kinova::Api::Base::CartesianTrajectoryConstraint::TypeCase::kSpeed:
	{
		m_Action.mutable_reach_pose()->mutable_constraint()->mutable_speed()->set_translation(constraint.speed().translation());
		m_Action.mutable_reach_pose()->mutable_constraint()->mutable_speed()->set_orientation(constraint.speed().orientation());
	}
	default:
	{
		// Do Nothing
	}
	}

	try
	{
		m_pBase->ExecuteAction(m_Action);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return true;
}

bool KortexRobot::SetJointAngles(const std::vector<float> &angles, const Kinova::Api::Base::JointTrajectoryConstraint &constraints)
{
	if (!m_bIsConnected || m_bIsBusy)
		return false;

	if (static_cast<int>(angles.size()) != m_nNbDoF)
		return false;

	m_Action.mutable_handle()->set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
	Kinova::Api::Base::ConstrainedJointAngles *constrainedJoint = m_Action.mutable_reach_joint_angles();
	Kinova::Api::Base::JointAngles *jointAngles = constrainedJoint->mutable_joint_angles();

	for (int a = 0; a < m_nNbDoF; a++)
	{
		Kinova::Api::Base::JointAngle *jointAngle = jointAngles->add_joint_angles();
		jointAngle->set_joint_identifier(a);
		jointAngle->set_value(angles[a]);
	}

	if (constraints.type() != Kinova::Api::Base::JointTrajectoryConstraintType::UNSPECIFIED_JOINT_CONSTRAINT)
	{
		constrainedJoint->mutable_constraint()->set_type(constraints.type());
		constrainedJoint->mutable_constraint()->set_value(constraints.value());
	}

	try
	{
		m_pBase->ExecuteAction(m_Action);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return true;
}

bool KortexRobot::SendTwistCommand(const tCartesianVector &translation, const tCartesianVector &rotation)
{
	if (!m_bIsConnected) // isBusy to avoid interruption as twist types have priority
		return false;

	Kinova::Api::Base::TwistCommand cmd;
	cmd.mutable_twist()->set_linear_x(translation.x);
	cmd.mutable_twist()->set_linear_y(translation.y);
	cmd.mutable_twist()->set_linear_z(translation.z);
	cmd.mutable_twist()->set_angular_x(rotation.x);
	cmd.mutable_twist()->set_angular_y(rotation.y);
	cmd.mutable_twist()->set_angular_z(rotation.z);

	try
	{
		m_pBase->SendTwistCommand(cmd);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}

	return true;
}

bool KortexRobot::SetTwistReferenceFrame(const Kinova::Api::Common::CartesianReferenceFrame &frame)
{
	if (!m_bIsConnected) // isBusy to avoid interruption as twist types have priority
		return false;

	Kinova::Api::ControlConfig::CartesianReferenceFrameInfo frameRequest;
	frameRequest.set_reference_frame(frame);

	// Kinova::Api::Common::CartesianReferenceFrame::
	// CARTESIAN_REFERENCE_FRAME_TOOL
	// CARTESIAN_REFERENCE_FRAME_BASE
	// CARTESIAN_REFERENCE_FRAME_MIXED
	// CARTESIAN_REFERENCE_FRAME_UNSPECIFIED

	try
	{
		m_pControlConfigClient->SetCartesianReferenceFrame(frameRequest);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}

	return true;
}

bool KortexRobot::SetJointSpeeds(std::vector<float> &jointSpeeds)
{
	if (!m_bIsConnected) // isBusy to avoid interruption as twist types have priority
		return false;

	if (static_cast<int>(jointSpeeds.size()) != m_nNbDoF)
		return false;

	Kinova::Api::Base::JointSpeeds speeds; // I think this needs to be a pointer definition? * speeds originally speeed
	for (int a = 0; a < m_nNbDoF; a++)
	{
		Kinova::Api::Base::JointSpeed *speed = speeds.add_joint_speeds(); // the above definition of speeds causes an issue here
		speed->set_joint_identifier(a);
		speed->set_value(jointSpeeds[a]);
	}
	try
	{
		m_pBase->SendJointSpeedsCommand(speeds);
		// If the above definition of speeds is changed to a pointer this produces an error: i can eliminate the error
		// should this be dereferenced (* speeds)? originally speeds
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}

	return true;
}

bool KortexRobot::PlaySequence(const Kinova::Api::Base::SequenceHandle &sequenceHandle)
{
	if (!m_bIsConnected || m_bIsBusy)
		return false;

	try
	{
		m_pBase->PlaySequence(sequenceHandle);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}

	return true;
}

bool KortexRobot::ExecuteAction(const Kinova::Api::Base::Action &action)
{
	if (!m_bIsConnected || m_bIsBusy)
		return false;

	try
	{
		m_pBase->ExecuteAction(action);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}

	return true;
}

void KortexRobot::PrintFeedback(int feedbackType)
{
	RefreshFeedback();
	switch (feedbackType)
	{
	case (FB_TYPE_JOINT_P):
	{
		for (int i = 0; i < m_nNbDoF; i++)
		{
			std::cout << "p_" << i << ": " << m_Feedback.actuators(i).position() << "\t\t"; // purposeful no end;
		}
		std::cout << std::endl;
		break;
	}
	case (FB_TYPE_JOINT_V):
	{
		for (int i = 0; i < m_nNbDoF; i++)
		{
			std::cout << "v_" << i << ": " << m_Feedback.actuators(i).velocity() << "\t\t"; // purposeful no end;
		}
		std::cout << std::endl;
		break;
	}
	case (FB_TYPE_JOINT_T):
	{
		for (int i = 0; i < m_nNbDoF; i++)
		{
			std::cout << "t_" << i << ": " << m_Feedback.actuators(i).torque() << "\t\t"; // purposeful no end;
		}
		std::cout << std::endl;
		break;
	}
	case (FB_TYPE_TOOL_P):
	{
		Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();
		std::cout << "X: " << bf.tool_pose_x()
				  << "\t\t Y: " << bf.tool_pose_y()
				  << "\t\t Z: " << bf.tool_pose_z()
				  << "\t\t Alpha: " << bf.tool_pose_theta_x()
				  << "\t\t Beta: " << bf.tool_pose_theta_y()
				  << "\t\t Gamma: " << bf.tool_pose_theta_z()
				  << std::endl;
		break;
	}
	case (FB_TYPE_TOOL_V):
	{
		Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();
		std::cout << "X': " << bf.tool_twist_linear_x()
				  << "\t\t Y': " << bf.tool_twist_linear_y()
				  << "\t\t Z': " << bf.tool_twist_linear_z()
				  << "\t\t Alpha': " << bf.tool_twist_angular_x()
				  << "\t\t Beta': " << bf.tool_twist_angular_y()
				  << "\t\t Gamma': " << bf.tool_twist_angular_z()
				  << std::endl;
		break;
	}
	case (FB_TYPE_TOOL_FT):
	{
		Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();
		std::cout << "F_X: " << bf.tool_external_wrench_force_x()
				  << "\t\t F_Y: " << bf.tool_external_wrench_force_y()
				  << "\t\t F_Z: " << bf.tool_external_wrench_force_z()
				  << "\t\t T_Alpha: " << bf.tool_external_wrench_torque_x()
				  << "\t\t T_Beta: " << bf.tool_external_wrench_torque_y()
				  << "\t\t T_Gamma: " << bf.tool_external_wrench_torque_z()
				  << std::endl;
		break;
	}
	case (FB_TYPE_NONE):
	{
		for (int i = 0; i < m_nNbDoF; i++)
		{
			if (!m_CustomData.empty())
			{
				// // collision detection for static pose: use gravity torque model of the robot and dynamic
				// // torque for required motion to estimate dynamic forces
				if (fabsf(m_Feedback.actuators(i).torque() - m_CustomData[i]) > 1.0f)
					std::cout << "Collision Detected on Joint" << i << std::endl;
			}
		}
		break;
	}
	default:
	{
		// no print
	}
	}
}

bool KortexRobot::WaitWhileRobotIsMoving(const int timeout)
{
	if (!m_bIsConnected)
		return false;

	int timeMs = 0;
	int timestepMs = 100;
	while (m_bIsBusy && timeMs < timeout)
	{
		RefreshFeedback();
		std::this_thread::sleep_for(std::chrono::milliseconds(timestepMs)); // wait some time to make sure it has arrived.
		PrintFeedback(FB_TYPE_TOOL_P);
		timeMs += timestepMs;
	}

	if (timeMs >= timeout)
		return false;

	return true;
}

bool KortexRobot::WaitWhileRobotIsMoving_Twist(const int timeout)
{
	if (!m_bIsConnected)
		return false;

	int timeMs = 0;
	int timestepMs = 100;
	while (timeMs < timeout)
	{
		RefreshFeedback();
		std::this_thread::sleep_for(std::chrono::milliseconds(timestepMs)); // wait some time to make sure it has arrived.
		PrintFeedback(FB_TYPE_NONE);
		timeMs += timestepMs;
	}

	if (timeMs >= timeout)
		return false;

	return true;
}

bool KortexRobot::Stop()
{
	if (!m_bIsConnected)
		return false;

	try
	{
		m_pBase->Stop();
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
	}

	return true;
}

Kinova::Api::BaseCyclic::ActuatorFeedback KortexRobot::GetActuatorFeedback(const int actIdx) const
{
	if (actIdx >= m_nNbDoF)
		return Kinova::Api::BaseCyclic::ActuatorFeedback();

	return m_Feedback.actuators(actIdx);
}

Kinova::Api::Base::Pose KortexRobot::GetCartesianPose() const
{
	Kinova::Api::Base::Pose pose;

	if (!m_bIsConnected)
		return pose;

	Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();
	pose.set_x(bf.tool_pose_x());
	pose.set_y(bf.tool_pose_y());
	pose.set_z(bf.tool_pose_z());
	pose.set_theta_x(bf.tool_pose_theta_x());
	pose.set_theta_y(bf.tool_pose_theta_y());
	pose.set_theta_z(bf.tool_pose_theta_z());

	return pose;
}

Kinova::Api::Base::Twist KortexRobot::GetCartesianTwist() const
{
	Kinova::Api::Base::Twist twist;

	if (!m_bIsConnected)
		return twist;

	Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();
	twist.set_linear_x(bf.tool_twist_linear_x());
	twist.set_linear_y(bf.tool_twist_linear_y());
	twist.set_linear_z(bf.tool_twist_linear_z());
	twist.set_angular_x(bf.tool_twist_angular_x());
	twist.set_angular_y(bf.tool_twist_angular_y());
	twist.set_angular_z(bf.tool_twist_angular_z());

	return twist;
}

Kinova::Api::Base::Wrench KortexRobot::GetCartesianWrench() const
{
	Kinova::Api::Base::Wrench wrench;

	if (!m_bIsConnected)
		return wrench;

	Kinova::Api::BaseCyclic::BaseFeedback bf = m_Feedback.base();
	wrench.set_force_x(bf.tool_external_wrench_force_x());
	wrench.set_force_y(bf.tool_external_wrench_force_y());
	wrench.set_force_z(bf.tool_external_wrench_force_z());
	wrench.set_torque_x(bf.tool_external_wrench_torque_x());
	wrench.set_torque_y(bf.tool_external_wrench_torque_y());
	wrench.set_torque_z(bf.tool_external_wrench_torque_z());

	return wrench;
}

std::vector<float> KortexRobot::GetJointPositions() const
{
	std::vector<float> positions;
	if (!m_bIsConnected)
		return positions;

	for (int idx = 0; idx < m_nNbDoF; idx++)
	{
		positions.push_back(m_Feedback.actuators(idx).position());
	}

	return positions;
}

std::vector<float> KortexRobot::GetJointVelocities() const
{
	std::vector<float> velocities;
	if (!m_bIsConnected)
		return velocities;

	for (int idx = 0; idx < m_nNbDoF; idx++)
	{
		velocities.push_back(m_Feedback.actuators(idx).velocity());
	}
	return velocities;
}

std::vector<float> KortexRobot::GetJointTorques() const
{
	std::vector<float> torques;
	if (!m_bIsConnected)
		return torques;

	for (int idx = 0; idx < m_nNbDoF; idx++)
	{
		torques.push_back(m_Feedback.actuators(idx).torque());
	}

	return torques;
}

bool KortexRobot::RefreshFeedback()
{
	if (!m_bIsConnected)
		return false;

	try
	{
		auto lambda_fct_callback = [this](const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback &data)
		{
			OnRefreshFeedbackCallback(err, data);
		};
		m_pBaseCyclic->RefreshFeedback_callback(lambda_fct_callback);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
		return false;
	}

	return true;
}

bool KortexRobot::SetCustomData(std::vector<float> data)
{
	if (static_cast<int>(data.size()) == m_nNbDoF)
	{
		m_CustomData = data;
		return true;
	}

	std::cout << "Invalid data" << std::endl;
	return false;
}

bool KortexRobot::MoveGripper(float vel)
{
	if (!m_bIsConnected)
		return false;

	Kinova::Api::Base::Gripper gripper_feedback;
	Kinova::Api::Base::GripperCommand gripper_command;
	Kinova::Api::Base::GripperRequest gripper_request;

	gripper_command.set_mode(Kinova::Api::Base::GRIPPER_SPEED);
	gripper_request.set_mode(Kinova::Api::Base::GRIPPER_SPEED);

	auto finger = gripper_command.mutable_gripper()->add_finger();
	finger->set_finger_identifier(1);
	finger->set_value(vel);
	m_pBase->SendGripperCommand(gripper_command);

	bool is_motion_completed = false;
	while (!is_motion_completed)
	{
		float speed = 0.0;
		gripper_feedback = m_pBase->GetMeasuredGripperMovement(gripper_request);
		if (gripper_feedback.finger_size())
		{
			speed = gripper_feedback.finger(0).value();
			cout << "Reported speed : " << speed << std::endl;
		}

		if (speed == 0.0f)
		{
			is_motion_completed = true;
		}
	}
	return true;
}

bool KortexRobot::forKin()
{
	// Current arm's joint angles
	Kinova::Api::Base::JointAngles input_joint_angles;
	try
	{
		std::cout << "Getting Angles for every joint..." << std::endl;
		input_joint_angles = m_pBase->GetMeasuredJointAngles();
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		std::cout << "Unable to get joint angles" << std::endl;
		OnError(ex);
		return false;
	}

	std::cout << "Joint ID : Joint Angle" << std::endl;
	for (auto joint_angle : input_joint_angles.joint_angles())
	{
		std::cout << joint_angle.joint_identifier() << " : " << joint_angle.value() << std::endl;
	}
	std::cout << std::endl;

	// Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
	Kinova::Api::Base::Pose pose;
	try
	{
		std::cout << "Computing Foward Kinematics using joint angles..." << std::endl;
		pose = m_pBase->ComputeForwardKinematics(input_joint_angles);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		std::cout << "Unable to compute forward kinematics" << std::endl;
		OnError(ex);
		return false;
	}

	std::cout << "Pose calculated : " << std::endl;
	std::cout << "Coordinate (x, y, z)  : (" << pose.x() << ", " << pose.y() << ", " << pose.z() << ")" << std::endl;
	std::cout << "Theta (theta_x, theta_y, theta_z)  : (" << pose.theta_x() << ", " << pose.theta_y() << ", " << pose.theta_z() << ")" << std::endl
			  << std::endl;

	return true;
}

bool KortexRobot::invKin()
{
	// get robot's pose (by using forward kinematics)
	Kinova::Api::Base::JointAngles input_joint_angles;
	Kinova::Api::Base::Pose pose;
	try
	{
		input_joint_angles = m_pBase->GetMeasuredJointAngles();
		pose = m_pBase->ComputeForwardKinematics(input_joint_angles);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		std::cout << "Unable to get current robot pose" << std::endl;
		OnError(ex);
		return false;
	}

	// Object containing cartesian coordinates and Angle Guess
	Kinova::Api::Base::IKData input_IkData;

	// Fill the IKData Object with the cartesian coordinates that need to be converted
	input_IkData.mutable_cartesian_pose()->set_x(pose.x());
	input_IkData.mutable_cartesian_pose()->set_y(pose.y());
	input_IkData.mutable_cartesian_pose()->set_z(pose.z());
	input_IkData.mutable_cartesian_pose()->set_theta_x(pose.theta_x());
	input_IkData.mutable_cartesian_pose()->set_theta_y(pose.theta_y());
	input_IkData.mutable_cartesian_pose()->set_theta_z(pose.theta_z());

	// Fill the IKData Object with the guessed joint angles
	Kinova::Api::Base::JointAngle *jAngle;
	for (auto joint_angle : input_joint_angles.joint_angles())
	{
		jAngle = input_IkData.mutable_guess()->add_joint_angles();
		// '- 1' to generate an actual "guess" for current joint angles
		jAngle->set_value(joint_angle.value() - 1);
	}

	// Computing Inverse Kinematics (cartesian -> Angle convert) from arm's current pose and joint angles guess
	Kinova::Api::Base::JointAngles computed_joint_angles;
	try
	{
		std::cout << "Computing Inverse Kinematics using joint angles and pose..." << std::endl;
		computed_joint_angles = m_pBase->ComputeInverseKinematics(input_IkData);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		std::cout << "Unable to compute inverse kinematics" << std::endl;
		OnError(ex);
		return false;
	}

	std::cout << "Joint ID : Joint Angle" << std::endl;
	int joint_identifier = 0;
	for (auto joint_angle : computed_joint_angles.joint_angles())
	{
		std::cout << joint_identifier << " : " << joint_angle.value() << std::endl;
		joint_identifier++;
	}

	return true;
}

bool KortexRobot::SetupLowLevelControl()
{
	bool return_status = true;
	auto servoingMode = Kinova::Api::Base::ServoingModeInformation();

	std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
	try
	{
		// Set the base in low-level servoing mode
		servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
		m_pBase->SetServoingMode(servoingMode);

		//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		m_Feedback = m_pBaseCyclic->RefreshFeedback();

		for (int i = 0; i < m_nNbDoF; i++)
		{	
			float thisPos = m_Feedback.actuators(i).position();
			m_Commands.push_back(thisPos);
			m_BaseCyclicCommand.add_actuators()->set_position(thisPos);
		}
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
		return_status = false;
	}
	catch (std::runtime_error &ex2)
	{
		std::cout << "Runtime error: " << ex2.what() << std::endl;
		return_status = false;
	}

	return return_status;
}

int64_t KortexRobot::GetTickUs()
{
#if defined(_MSC_VER)
	LARGE_INTEGER start, frequency;

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);

	return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
	struct timespec start;
	clock_gettime(CLOCK_MONOTONIC, &start);

	return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

bool KortexRobot::LowLevelMove(std::vector<float> jointpos)
{
	auto lambda_fct_callback = [](const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            //std::cout << serialized_data << std::endl << std::endl;
        };

	try
	{
		for (uint8_t i = 0; i < m_nNbDoF; i++)
		{
			m_Commands[i] = jointpos[i];		
			float thisPos = fmod(m_Commands[i], 360.0f); // this computes the new poisition based on incoming velocity....
			m_BaseCyclicCommand.mutable_actuators(i)->set_position(thisPos); // loop over after 360deg.
			//std::cout << "Setting joint " << i << " to " << thisPos << " degrees" << std::endl;
		}
	
		m_pBaseCyclic->Refresh_callback(m_BaseCyclicCommand, lambda_fct_callback, 0);
	}
	catch (Kinova::Api::KDetailedException &ex)
	{
		OnError(ex);
		return false;
	}
	return true;
}