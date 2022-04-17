% Install ROS Toolbox on Matlab
% Type "roslaunch rplidar_ros rplidar.launch" on Pi CLI
% Run plot_lidar.m on Matlab GUI
% run command "rosshutdown" once done to shutdown ros network

IP_ADDRESS = '172.20.10.10'

rosinit(IP_ADDRESS);   
linkdata on

% Polls for LIDAR '/scan' data
% If error occurs, error type is printed to the command window
% Else the data is plotted on an axis

while 1
    try
        sub = rossubscriber('/scan'); % suscribes to LIDAR data
        scan = receive(sub);
        plot(scan,'MaximumRange', 2);
        rectangle('Position', [-0.025 -0.045 0.19 0.10]); % rectangle to simulate dimensions of Alex 
    catch e 
        fprintf(1,'The identifier was:\n%s',e.identifier);
        fprintf(1,'There was an error! The message was:\n%s',e.message);
    end
end
