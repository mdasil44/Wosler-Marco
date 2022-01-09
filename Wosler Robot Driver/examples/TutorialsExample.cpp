#include "Classes/include/KortexRobot.hpp"

int main(int argc, char** argv) {
	KortexRobot robot = KortexRobot("192.168.1.10");

	if (robot.IsConnected()) {
		robot.SubscribeToNotifications();
		
		std::cout << "Going Home" << std::endl;
		auto action_type = Kinova::Api::Base::RequestedActionType();
		//Execute Existing Action "Home"
		action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
		robot.ExecuteExistingAction("Home", action_type);
		robot.WaitWhileRobotIsMoving(10000);

		// std::vector<float> jointTorques = robot.GetJointTorques();
		// if (!robot.SetCustomData(jointTorques)) // gets nominal joint torques in the home position 
		// 	return 0;

		//Cartesian Action
		std::cout << "Cartesian Action" << std::endl;
		Kinova::Api::Base::CartesianTrajectoryConstraint constraint = Kinova::Api::Base::CartesianTrajectoryConstraint();
		constraint.mutable_speed()->set_translation(0.25f);
		constraint.mutable_speed()->set_orientation(30.0f);
		robot.MoveTo(tCartesianVector(0.35f, 0.0f, 0.15f), tCartesianVector(179.0f, 0.0f, 0.0f), constraint);
		robot.WaitWhileRobotIsMoving(10000); // 10s

		//Angular Action
		std::cout << "Jont Angle Action" << std::endl;
		std::vector<float> pos1 {0.0f, 90.0f, 90.0f, 90.0f, 0.0f, 90.0f, 0.0f}; 
		robot.SetJointAngles(pos1, Kinova::Api::Base::JointTrajectoryConstraint());
		robot.WaitWhileRobotIsMoving(10000); // 10s

		std::cout << "Tool Velocity Action" << std::endl;
		//Twist Commands
		robot.SetTwistReferenceFrame(Kinova::Api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL);
		robot.SendTwistCommand(tCartesianVector(0.0,0.0f,0.1f), tCartesianVector(15.0f,0.0f,0.0f));
		robot.WaitWhileRobotIsMoving_Twist(4000);
		robot.Stop();

		std::cout << "Joint Velocity Action" << std::endl;
		//Joint Speeds
		std::vector<float> speeds (6, 0.0f); 
		speeds.push_back(40.0f);
		robot.SetJointSpeeds(speeds);
		robot.WaitWhileRobotIsMoving_Twist(4000);
		robot.Stop();

		std::cout << "Stand up straight" << std::endl;
		std::vector<float> pos2 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
		robot.SetJointAngles(pos2, Kinova::Api::Base::JointTrajectoryConstraint());
		robot.WaitWhileRobotIsMoving(10000); // 10s

		std::cout << "Cartesian Action" << std::endl;
		constraint = Kinova::Api::Base::CartesianTrajectoryConstraint();
		constraint.mutable_speed()->set_translation(0.20f);
		constraint.mutable_speed()->set_orientation(30.0f);
		robot.MoveTo(tCartesianVector(0.35f, 0.0f, 0.4f), tCartesianVector(180.0f, 90.0f, 0.0f), constraint);
		robot.WaitWhileRobotIsMoving(10000); // 10s

		std::cout << "Back to Home" << std::endl;
		action_type = Kinova::Api::Base::RequestedActionType();
		action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
		robot.ExecuteExistingAction("Home", action_type);
		robot.WaitWhileRobotIsMoving(10000);

		robot.UnsubscribeToNotifications();
	}

	std::cout << "End of Program" << std::endl;
	return 0;
}