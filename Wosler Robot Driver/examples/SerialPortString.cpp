
// Serial library
#include "Classes/include/serialib.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <unistd.h>

#define SERIAL_PORT "\\\\.\\COM4"

char newChar;
char *newCharPtr = &newChar;

std::string newDataString = "                                                 ";
char *newDataPtr;

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

int main()
{
    // Serial object
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
    while (serial.available() == 0);
    serial.readChar(newCharPtr);
    if (newChar == 'S')
        std::cout << "Connection successful" << std::endl;
    serial.flushReceiver();
    //serial.writeChar('1');


    int timeMs = 0;
    int timeout = 5000;
    while (timeMs < timeout)
    {   
        if(serial.available()>10)
        {
            newDataPtr = &newDataString[0];
            serial.readString(newDataPtr, '\n', 48, 10);
            std::vector<float> data = split(&newDataString[0],',');
            std::cout << data[0] << '\t' << data[1] << '\t' << data[2] << '\t' << data[3]  << '\t' << data[4] << '\t' << data[5] << std::endl;
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
    return 0;
}
