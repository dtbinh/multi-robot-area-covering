# multi-robot-area-covering

This project is trying to use multi-robot cooperation to cover an area. Two robots are working at the beginning and the other one is charging and waiting. Two robots start working from two sides. When one of them is running out of charge. It will send its current location to master and master will tell Robot 3 where to go and where to stop. After Robot 3 goes to the position where another robot left, it takes that one's job and continues area covering in zigzag format.

In each slave's program, “imu.h” deals with the I2C communication between absolute orientation IMU and mbed and the data reading for heading. “zigbee.h” includes functions that receive and send data using MRF24J40 module. “main.cpp” contains all the logic and other functions.
