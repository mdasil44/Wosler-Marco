#include "Classes/include/KortexRobot.hpp"
#include "Classes/include/serialib.hpp"
//#include <unistd.h>

#define SERIAL_PORT "\\\\.\\COM4"

char newChar;
char *newCharPtr = &newChar;
const std::string resetString = "                                                 ";
std::string newDataString = resetString;
char *newDataPtr = &newDataString[0];

float xd,yd,zd,txd,tyd,tzd;

std::vector<float> split(const std::string& s, char sep)
{
    std::vector<float> output;
    std::size_t start = 0, end = 0;
    std::string subSt;
    while ((end = s.find(sep, start)) != std::string::npos) 
    {
        subSt = s.substr(start, end-start);
        output.push_back(std::stof(subSt));
        start = end + 1;
    }
    subSt = s.substr(start, end-start);
    output.push_back(std::stof(subSt));

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
	while(1){
		if (serial.available() > 0){
			serial.readChar(newCharPtr);
			if (newChar == 'S'){
				std::cout << "Connection successful" << std::endl;
				serial.flushReceiver();
				break;
			}
		}
	}

	if (robot.IsConnected())
	{
		robot.SubscribeToNotifications();
		//robot.MoveGripper(-0.5);
		//std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		std::cout << "Going Home" << std::endl;
		auto action_type = Kinova::Api::Base::RequestedActionType();
		action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
		robot.ExecuteExistingAction("Home", action_type);
		robot.WaitWhileRobotIsMoving(10000); // 10s
		
		//cin.get();
		Kinova::Api::Base::CartesianTrajectoryConstraint constraint = Kinova::Api::Base::CartesianTrajectoryConstraint();
		constraint.mutable_speed()->set_translation(0.25f);
		constraint.mutable_speed()->set_orientation(30.0f);
		robot.MoveTo(tCartesianVector(0.25f, 0.10f, 0.30f), tCartesianVector(-90.0f, -180.0f, 0.0f), constraint);
		robot.WaitWhileRobotIsMoving(10000); // 10s

		while(1){
			if (serial.available()>=10){
				newDataString = resetString;
				newDataPtr = &newDataString[0];
            	serial.readString(newDataPtr, '\n', 50, 10);
				serial.flushReceiver();
				std::cout << newDataString << '\n'; 
			
				if (newDataString[0] == 'R'){
					break;
				}
				
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		robot.SetTwistReferenceFrame(Kinova::Api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED);
		unsigned int timeoutClock = 0;
		unsigned int recalibrateClock = 0;
		unsigned int recalibrateTime = 5000;
		unsigned int timeout = 200;
		while (timeoutClock < timeout)
		{
			if (serial.available() >= 30)
			{
				newDataPtr = &newDataString[0];
            	serial.readString(newDataPtr, '\n', 50, 10);
				std::vector<float> data;

				if (newDataString[0] == 'v')
				{	
					data = split(&newDataString[1],',');
					robot.SendTwistCommand(tCartesianVector(0.001*data[0],0.001*data[1],0.001*data[2]), 
										tCartesianVector(0.0f, -data[3], data[4]));
					timeoutClock = 0;
					std::cout<< "Velocity: ";
				}
				else if (newDataString[0] == 'p')
				{
					data = split(&newDataString[1],',');
					// robot.MoveTo(tCartesianVector(0.45f + 0.001*data[0], 0.0f + 0.001*data[1], 0.3f + 0.001* data[2]),
					// 							tCartesianVector(-180.0f + data[4], 0.0f, 90.0f + data[5]), constraint);
					// robot.WaitWhileRobotIsMoving(2000); // 2s
					recalibrateClock = 0;
					std::cout<< "Position: ";
				}
				std::cout << data[0] << '\t' << data[1] << '\t' << data[2] << '\t' << data[3]  << '\t' << data[4] << '\t' << data[5] << '\n';
				serial.flushReceiver();
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			timeoutClock++;
			recalibrateClock++;

			if (recalibrateTime >= recalibrateClock)
				serial.writeChar('1');
		}
		std::cout << "Timeout! Closing program" << std::endl;
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