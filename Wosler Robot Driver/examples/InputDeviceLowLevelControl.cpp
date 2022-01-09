#include "Classes/include/KortexRobot.hpp"
#include "Classes/include/serialib.hpp"

#define SERIAL_PORT "\\\\.\\COM4"
#define STR_LEN 50
#define STR_TIMEOUT 10
#define KHZ 999

const std::string resetString(50, ' ');
std::string newDataString = resetString;
char *newDataPtr = &newDataString[0];

float alpha = 0.01;
std::vector<float> robotjointpos(0,7); // 6 dof for manual inversekin
std::vector<float> jointpos(0,7); // 6 dof for manual inversekin

std::vector<float> split(const std::string &s, char sep)
{
	std::vector<float> output;
	std::size_t start = 0, end = 0;
	std::string subSt;
	while ((end = s.find(sep, start)) != std::string::npos)
	{
		subSt = s.substr(start, end - start);
		output.push_back(std::stof(subSt));
		start = end + 1;
	}
	subSt = s.substr(start, end - start);
	output.push_back(std::stof(subSt));

	return output;
}

std::vector<float> lowpass(KortexRobot robot, std::vector<float> jointpos,std::vector<float> robotjointpos)
{
	for (uint8_t i = 0; i < robot.GetNbDoF(); i++) 
	{	
		float newval = alpha*jointpos[i] + (1-alpha)*robotjointpos[i];
		robotjointpos.push_back(newval);
	}
	return robotjointpos;
}

void dataCollection(serialib serial)
{
	while (1)
	{
		// 	"T matrix" from the input device 
		//	r11	r12	r13
		// 	r21	r22	r23
		//  r31	r32	r33
		// 	x  	y  	z

		if (serial.available() > 30)
		{
			newDataPtr = &newDataString[0];
			serial.readString(newDataPtr, '\n', STR_LEN, STR_TIMEOUT);
			std::vector<float> data;

			if (newDataString[0] == 'p')
			{
				data = split(&newDataString[1], ',');
				std::cout << "Position Setpoints: ";
			}
			
			std::cout << data[0] << ',' << data[1] << ',' << data[2] << ',' << data[3] << ',' << data[4] << ',' << data[5] << '\n';
			serial.flushReceiver();
			
			//jointpos = robot.invkinT(T);
			//mutex(jointpos);
			//mutex flag high;
		}
	}
}

void ClosedLoopPositionControl(KortexRobot robot)
{
	uint64_t now = 0;
	uint64_t last = 0; // active for about 585000 years.
	while(1)
	{
		now = robot.GetTickUs();
		if (now - last > KHZ)
		{
			//mutex flag low;
			robotjointpos = lowpass(robot,jointpos,robotjointpos);
			 
			robot.LowLevelMove(robotjointpos);
			last = now;
		}
	}
}

int main(int argc, char **argv)
{
	// Setup system objects
	KortexRobot robot = KortexRobot("192.168.1.10");
	serialib serial;

	// Establish Connection
	std::cout << "Attempting to connect" << std::endl;
	int errorOpening = serial.openDevice(SERIAL_PORT, 115200);
	if (errorOpening != 1)
	{
		std::cout << "Error Code: " << errorOpening << ": Connection unsuccessful, serial port was not found." << std::endl;
		return errorOpening;
	}

	// Handshake
	serial.writeChar('H');
	while (newDataString[0] != 'S')
	{
		if (serial.available() > 0)
		{
			newDataString = resetString;
			newDataPtr = &newDataString[0];
			serial.readString(newDataPtr, '\n', STR_LEN, STR_TIMEOUT);
			serial.flushReceiver();
		}
	}
	std::cout << "Connection successful" << std::endl;
	serial.flushReceiver();

	// prepare robot for action
	if (robot.IsConnected())
	{
		// Move Robot to Ready Position
		robot.SubscribeToNotifications();
		robot.MoveGripper(-0.5);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		std::cout << "Going Home" << std::endl;
		auto action_type = Kinova::Api::Base::RequestedActionType();
		action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
		robot.ExecuteExistingAction("Home", action_type);
		robot.WaitWhileRobotIsMoving(10000); // 10s

		Kinova::Api::Base::CartesianTrajectoryConstraint constraint = Kinova::Api::Base::CartesianTrajectoryConstraint();
		constraint.mutable_speed()->set_translation(0.25f);
		constraint.mutable_speed()->set_orientation(30.0f);
		robot.MoveTo(tCartesianVector(0.25f, 0.10f, 0.30f), tCartesianVector(-90.0f, -180.0f, 0.0f), constraint);
		robot.WaitWhileRobotIsMoving(10000); // 10s

		// Wait for input device to be homed
		while (newDataString[0] != 'R')
		{
			if (serial.available() > 10)
			{
				newDataString = resetString;
				newDataPtr = &newDataString[0];
				serial.readString(newDataPtr, '\n', STR_LEN, STR_TIMEOUT);
				serial.flushReceiver();
				std::cout << newDataString << '\n';
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		robot.SetupLowLevelControl();

	// thread 1: Collect Data
	std::thread collectData(dataCollection, serial); // need to pass the serial object. 

	// thread 2: Closed Loop Position Control
	std::thread RobotRT(ClosedLoopPositionControl);


	collectData.join();
	RobotRT.join();

	std::cout << "Done! Closing program" << std::endl;
	serial.writeChar('R');

	delete newDataPtr;

	// Close the serial device
	serial.closeDevice();
	robot.UnsubscribeToNotifications();
}

std::cout << "End of Program" << std::endl;
return 0;
}
