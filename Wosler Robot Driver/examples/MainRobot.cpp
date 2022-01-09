// setup
    // establish communication
    // initiate robot
    // stage the robot in a start position
    // home the input device, once homed break 


// thread 1: collect data from input device
// measure the current forward kinematic model of the input device
// calculate the robot's demand joint positions based on the data
// once done share that joint positions to thread 2

    
// thread 2: recieve the joint positions calculate in thread 1 filter/interp/exterp (alpha < 0.01)
// apply these positions as the new setpoints for the robot position. use low level control
// loop at 1KHz
