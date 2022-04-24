# CG2111A

CG2111A, Engineering Principles and Practices II, covers the fundamental principles on certain advanced concepts and then design and programme a real-world system. The module involves designing a complex computer engineering system that facilitates information processing, real-world interfacing, and understanding the effects of certain useful metrics such as, scaling, safety, security, sustainability, societal impact, fault-tolerant design, etc. The final project of the module, a robot named Alex is a practical demonstration of the concepts learned throughout the module.

## Alex

Alex is a remotely controlled robot, capable of navigating simple unseen terrains whilst mapping the surrounding environment. The main controlling device of Alex is a Raspberry Pi 2 (RPi 2) installed with Robot Operating System (ROS) Kinetic. In addition, Alex also has a  Light Detection and Ranging (LiDAR) scanner, the Slamtech RPLIDAR A1 for environmental mapping, an Arduino UNO for motor control and two HC-SR04 Ultrasonic Sensors to aid in navigation.

Motor control is achieved through commands sent to Alex from a remote workstation, specifying the direction, distance or angle, and the power of the motors of Alex. This is achieved through a TLS Server-Client connection. The commands and parameters are sent from the client (remote workstation) to the server(RPi) utilising a TCP Socket. The RPi receives the commands and serialises it before transmitting the data to the Arduino Uno using a UART connection. The uno receives the data and carries out the respective the movements as specified by the operator.

The LiDAR data works utilising ROS and the rplidar_ros library. To visualise the data, we use the MATLAB GUI Application on a remote laptop to plot out the LiDAR scan data achieved by subscribing to the '/scan' message on the ROS Master. The subsequent plot represents a 360 degree 2D visualisation of the environment around Alex and is the main method used to navigate around unseen terrains.

The Ultrasonic Sensors provide additional data to aid in navigation and supplement any lack of information in the LiDAR data. This is achieved by utilising the RPi's GPIO pins and the pigpio header file to obtain two readings for each Ultrasonic sensor. The readings aid in navigation by providing more environmental data to aid in navigate around an unseen and unknown environment.


## Configuring ROS on Alex

The RPi should have ROS Kinetic as well the rplidar_ros package installed properly. 

To allow external connections to the ROS Master, the ROS_MASTER_URI environment variable must be set to the IP address of the RPi. This can be done by configuring the .bashrc file on the Pi.

For example if the IP address of the RPi is 192.158.1.38, we will run the following command:

```bash
echo 'export $ROS_MASTER_URI=http://192.158.1.38:11311' >> ~/.bashrc
```

## Movement control of Alex

First, the Arduino Uno must have the correct program uploaded. Connect a laptop to the Uno via USB and upload the file 'Alex.ino' to the Uno. Ensure the necessary header and zip files are included for the program to compile. 

Once done, we will set up the TLS server and client. Before this step, both the remote workstation as well as the RPi must have the necessary CA's as well as private and public keys to facilitate the TLS handshake.

To set up the TLS Server and get it running, navigate over to the directory containing the 'tls-alex-server.cpp' file in a new terminal on the RPi. 

Run the following commands:

```bash
g++ tls-alex-server.cpp tls_server_lib.cpp tls_pthread.cpp make_tls_server.cpp tls_common_lib.cpp serial.cpp serialize.cpp -pthread -lssl -lcrypto -o tls-alex-server

./tls-alex-server
```

Henceforth, the TLS server will be running and listening for an incoming connection.

To set up TLS Client, open a terminal on the remote workstation and navigate over to the folder containing 'tls-alex-client.cpp'. 

First we need to install standard TLS Programming Libraries using the following commands:

```bash
sudo apt-get install libssl-dev
```

Once done, we can compile and run the TLS Client. The TLS client requires the IP address of the RPi as well the port the server is running on. In this case it is port 5000.

Run the following commands to compile and run the client:

```bash
g++ tls-alex-client.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -pthread -lssl -lcrypto -o tls-alex-client

./tls-alex-client 192.158.1.38 5000
```
The terminal of the remote workstation will confirm the success of the TLS Handshake. Henceforth, the TLS Client is running and the command and parameters can be sent to Alex securely.
