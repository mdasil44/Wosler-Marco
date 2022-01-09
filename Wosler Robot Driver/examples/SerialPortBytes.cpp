
// Serial library
#include "Classes/include/serialib.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <algorithm>

#define SERIAL_PORT "\\\\.\\COM4"

char newChar;
char *newCharPtr = &newChar;

uint8_t byteArray[12];
int16_t data[6];
void *newDataPtr;
const uint8_t *thisBytePtr;

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

    serial.writeChar('1');
    int dataCount = 0;
    int maxCount = 100;
    while (dataCount < maxCount)
    {   
        if(serial.available()>=12)
        {
            newDataPtr = &byteArray[0];
            serial.readBytes(newDataPtr, 12, 10, 0);

            for (int i = 0; i < 6; i++)
            {
                int16_t msb = -unsigned(byteArray[2*i + 1]);
                int16_t lsb = -unsigned(byteArray[2*i]); 
                data[i] = -((msb << 8) | (lsb & 0xff));
                std::cout << data[i] << '\t';
            }
            std::cout << '\n';

            serial.flushReceiver();
            serial.writeChar('1');
            dataCount++;
        }   
    }
    std::cout << "Timout! Closing program" << std::endl;
    serial.writeChar('R');

    delete newCharPtr;
    delete newDataPtr;

    // Close the serial device
    serial.closeDevice();
    return 0;
}
