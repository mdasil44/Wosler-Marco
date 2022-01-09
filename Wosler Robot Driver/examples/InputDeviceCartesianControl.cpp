#include "Classes/include/KortexRobot.hpp"
#include "Classes/include/serialib.hpp"
//#include <unistd.h>

#define SERIAL_PORT "\\\\.\\COM4"

char newChar;
char *newCharPtr = &newChar;

std::string newDataString = "                                             ";
char *newDataPtr;

std::vector<int16_t> split(const std::string &s, char sep)
{
	std::vector<int16_t> output;
	std::size_t start = 0, end = 0;
	std::string subSt;
	while ((end = s.find(sep, start)) != std::string::npos)
	{
		subSt = s.substr(start, end - start);
		output.push_back(std::stoi(subSt));
		start = end + 1;
	}
	subSt = s.substr(start, end - start);
	output.push_back(std::stoi(subSt));

	return output;
}

int main(int argc, char **argv)
{
	KortexRobot robot = KortexRobot("192.168.1.10");
	serialib serial;

	// Connection to serial port
	std::cout << "Attempting to connect" << std::endl;
	int errorOpening = serial.openDevice(SERIAL_PORT, 115200);

	// If connection fails, return the error code otherwise, display a success message
	if (errorOpening != 1)
	{
		std::cout << "Error Code: " << errorOpening << ": Connection unsuccessful, serial port was not found." << std::endl;
		return errorOpening;
	}

	serial.writeChar('H');
	while (serial.available() == 0)
	{
		std::cout << ".";
	};
	serial.readChar(newCharPtr);
	if (newChar == 'S')
		std::cout << "Connection successful" << std::endl;

	serial.flushReceiver();

	if (robot.IsConnected())
	{
		robot.SubscribeToNotifications();

		std::cout << "Going Home" << std::endl;
		auto action_type = Kinova::Api::Base::RequestedActionType();
		// Execute Existing Action "Home"
		action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
		robot.ExecuteExistingAction("Home", action_type);
		robot.WaitWhileRobotIsMoving(10000);

		// Cartesian Action
		std::cout << "Cartesian Action" << std::endl;
		Kinova::Api::Base::CartesianTrajectoryConstraint constraint = Kinova::Api::Base::CartesianTrajectoryConstraint();
		constraint.mutable_speed()->set_translation(0.5f);
		constraint.mutable_speed()->set_orientation(50.0f);
		robot.MoveTo(tCartesianVector(0.45f, 0.0f, 0.30f), tCartesianVector(-180.0f, 0.0f, 90.0f), constraint);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		robot.WaitWhileRobotIsMoving(10000); // 10s

		robot.MoveGripper(0.3);

		//serial.writeChar('1');
		int timeMs = 0;
		int timeout = 5000;
		while (timeMs < timeout)
		{
			if (serial.available() > 10)
			{
				newDataPtr = &newDataString[0];
				serial.readString(newDataPtr, '\n', 45, 10);
				std::vector<int16_t> data = split(&newDataString[0], ',');
				std::cout << 0.45+0.001*data[0] << '\t' << 0.001*data[1] << '\t' << 0.3+0.001*data[2] << '\t' << data[3] << '\t' << data[4] << '\t' << data[5] << std::endl;

				robot.MoveTo(tCartesianVector(0.45 + 0.001*data[0], 0.001*data[1], 0.3 + 0.001*data[2]), tCartesianVector(-180.0 + data[4], 0.0f, 90.0f + data[5]), constraint);
				serial.flushReceiver();
				//serial.writeChar('1');
				timeMs = 0;
			}
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
			timeMs++;
		}
		std::cout << "Timout! Closing program" << std::endl;
		serial.writeChar('R');

		delete newCharPtr;
		delete newDataPtr;

		// Close the serial device
		serial.closeDevice();

		robot.UnsubscribeToNotifications();
	}

	std::cout << "End of Program" << std::endl;
	return 0;
}