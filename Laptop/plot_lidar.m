% Install ROS Toolbox on Matlab
% Type "roslaunch rplidar_ros rplidar.launch" on Pi CLI
% Run plot_lidar.m on Matlab GUI
% run command "rosshutdown" once done to shutdown ros network

rosinit('172.20.10.10'); %ROS_MASTER_URI aka Pi's IP address  
linkdata on %links data to plot for real time updates

while 1
    try
        sub = rossubscriber('/scan'); % suscribes to Lidar data
        scan = receive(sub);
        plot(scan,'MaximumRange', 2);
        rectangle('Position', [-0.025 -0.045 0.19 0.10]); % rectangle to simulate dimensions of Alex 
    catch e 
        fprintf(1,'The identifier was:\n%s',e.identifier);
        fprintf(1,'There was an error! The message was:\n%s',e.message);
    end
end