# Self-balancing-robot-using-Esp32-with-MPU-6050-address-0x69
this repository is about my course project ( making a mini segway )

There are only two libraries required for this project, wire.h and MPU6050_light by tockn\\
which can be downloaded from the arduino library manager 

Also if you are using a mpu 6050 with its slave address as 0x69 \\
then go to the library's header files and find the line #define MPU_ADDR 0x68 \\
and then change it to 0x69 or else MPU will not get initialized\\

There was also this problem, if I let the mpu take readings for a long time\\
say about 10 minutes then the mpu would suddenly fail to initialize\\
but after letting the system to rest for a few minutes mpu was able to start working just fine\\

finally if you find that the robot is not balancing for any value of kp, ki and kd try increasing\\
the moment of inertia of the robot, i.e. try to add more vertical levels to the robot\\

To understand more about the functioning of a self balancing robot I suggest you to watch some youtube videos on them\\
this file is just a compilation of all the challenges I faced while making my mini segway
