% Read CSV File to demostrate plot
% For PGM Log Analysis
% PO-JUI, CHIU    2021/08/25

clc; clear; close all;

%% File Designate
% Prefix file name -> time stamp
file_time_stamp = "11-30 22-3-17";
file_name_list = [" Controller"; "Altitude Velocity Controller"; "Altitude Sensor";
    "Altitude Sensor - Lidar"; "Position Controller"; "Velocity Controller"; "Heading Controller";
    "Camera Sensor"; "Command Transfer"]';

%% Load data
alt_ctrl_plot(file_time_stamp)

%alt_sens_plot(file_time_stamp)
%alt_sens_lidar_plot(file_time_stamp)
%cam_sens_plot(file_time_stamp)
pos_ctrl_plot(file_time_stamp)
hdg_ctrl_plot(file_time_stamp)
cmd_trans_plot(file_time_stamp)
position_state_plot(file_time_stamp)
%}
%% Altitude Control Plot
function alt_ctrl_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Altitude Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure()
    subplot(2,3,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),2:4))
    legend('Set Point','Response','Control Output')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Altitude Controller')
    hold off
    
    subplot(2,3,2)
    hold on
    grid on
    plot(time,data(1:sizes(1),5))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),5)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Altitude Controller Time Performance - Process')
    hold off
    
    subplot(2,3,3)
    hold on
    grid on
    plot(time,data(1:sizes(1),6))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),6)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Altitude Controller Time Performance - Period')
    hold off
    
    head = readmatrix(file_time_stamp+"_Altitude Velocity Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Velocity Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    subplot(2,3,4)
    hold on
    grid on
    stairs(time,data(1:sizes(1),2:4))
    legend('Set Point','Response','Control Output')
    xlabel('Time (sec)')
    ylabel('Amplitude (m/s, thrust scale)')
    title('Altitude Rate Controller')
    hold off
    
    subplot(2,3,5)
    hold on
    grid on
    plot(time,data(1:sizes(1),5))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),5)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Altitude Rate Controller Time Performance - Process')
    hold off
    
    subplot(2,3,6)
    hold on
    grid on
    plot(time,data(1:sizes(1),6))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),6)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('AltitudeRate  Controller Time Performance - Period')
    hold off
    
    head = readmatrix(file_time_stamp+"_Altitude Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure()
    subplot(1,2,1)
    hold on
    grid on
    plot(time, data(1:sizes(1), 4), 'LineWidth', 1.5)
    plot(time, data(1:sizes(1),[7 8 9]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Altitude Controller Separate Output')
    hold off
    
    head = readmatrix(file_time_stamp+"_Altitude Velocity Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Velocity Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    subplot(1,2,2)
    hold on
    grid on
    plot(time, (data(1:sizes(1), 4)), 'LineWidth', 1.5)
    plot(time, data(1:sizes(1),[7 8 9]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Altitude Rate Controller Separate Output')
    hold off

end

%% Altitude Sensor Plot
function alt_sens_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Altitude Sensor.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Sensor.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure()
    subplot(2,2,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),2:3))
    legend('Relative','Absolute')
    xlabel('Time (sec)')
    ylabel('Amplitude (m)')
    title('Altitude Sensor')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    stairs(time,data(1:sizes(1),4:5))
    legend('Relative','Absolute')
    xlabel('Time (sec)')
    ylabel('Amplitude (m/s)')
    title('Altitude Rate Sensor')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(time,data(1:sizes(1),6))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),6)),'-','LineWidth',1.5)
    legend('Measurement','Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Altitude Sensor Time Performance - Measurement')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(time,data(1:sizes(1),7))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),7)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Altitude Sensor Time Performance - Period')
    hold off
    
end

%% Altitude Sensor - Lidar Plot
function alt_sens_lidar_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Altitude Sensor - Lidar.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Sensor - Lidar.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure()
    subplot(2,2,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),2:4))
    legend(head(1,2:4))
    xlabel('Time (sec)')
    ylabel('Amplitude (m)')
    title('Altitude Sensor - Lidar')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    stairs(time,data(1:sizes(1),5))
    legend('Strength')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Altitude Sensor - Lidar Strength')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(time,data(1:sizes(1),6))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),6)),'-','LineWidth',1.5)
    legend('Update Rate','Mean')
    xlabel('Time (sec)')
    ylabel('Frequency (Hz)')
    title('LiDAR Time Performance - Update Rate')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(time,data(1:sizes(1),8))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),8)),'-','LineWidth',1.5)
    legend('Measurement', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('LiDAR Time Performance - Measurement')
    hold off
    
end

%% Camera Sensor Plot
function cam_sens_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Camera Sensor.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Camera Sensor.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure()
    subplot(3,2,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),[4 6]))
    legend('X','Vx')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Camera Sensor - X State')
    hold off
    
    subplot(3,2,2)
    hold on
    grid on
    stairs(time,data(1:sizes(1),[5 7]))
    legend('Y','Vy')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Camera Sensor - Y State')
    hold off
    
    subplot(3,2,3)
    hold on
    grid on
    stairs(time,data(1:sizes(1),8))
    legend('Angle')
    xlabel('Time (sec)')
    ylabel('Amplitude (degree)')
    title('Camera Sensor - Angle State')
    hold off
    
    subplot(3,2,4)   
    hold on
    grid on
    stairs(time,data(1:sizes(1),9))
    legend('Area')
    xlabel('Time (sec)')
    ylabel('Amplitude (m^2)')
    title('Camera Sensor - Area State')
    hold off
    
    subplot(3,2,5)
    hold on
    grid on
    plot(time,data(1:sizes(1),10))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),10)),'-','LineWidth',1.5)
    legend('Measurement','Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Camera Sensor Time Performance - Measurement')
    hold off
    
    subplot(3,2,6)
    hold on
    grid on
    plot(time,data(1:sizes(1),11))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),11)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Camera Sensor Time Performance - Period')
    hold off
    
end

%% Position Control Plot
function pos_ctrl_plot(file_time_stamp)
    p_head = readmatrix(file_time_stamp+"_Position Controller.csv",'Range','1:1','OutputType','string');
    p_data = readmatrix(file_time_stamp+"_Position Controller.csv");
    p_time = p_data(:,1);
    p_sizes = size(p_data);
    
    v_head = readmatrix(file_time_stamp+"_Velocity Controller.csv",'Range','1:1','OutputType','string');
    v_data = readmatrix(file_time_stamp+"_Velocity Controller.csv");
    v_time = v_data(:,1);
    v_sizes = size(v_data);
    
    figure()
    subplot(2,2,1)
    hold on
    grid on
    stairs(p_time,p_data(1:p_sizes(1),[2 4 6]))
    legend('Set Point','Response','Control Output')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Position Controller - X')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    stairs(p_time,p_data(1:p_sizes(1),[3 5 7]))
    legend('Set Point','Response','Control Output')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Position Controller - Y')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(p_time,p_data(1:p_sizes(1),8))
    plot(p_time, ones(1,p_sizes(1)).*mean(p_data(1:p_sizes(1),8)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Position Controller Time Performance - Process')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(p_time,p_data(1:p_sizes(1),9))
    plot(p_time, ones(1,p_sizes(1)).*mean(p_data(1:p_sizes(1),9)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Position Controller Time Performance - Period')
    hold off
    
    figure()
    subplot(2,2,1)
    hold on
    grid on
    stairs(v_time,v_data(1:v_sizes(1),[2 4 7 9]))
    legend('Set Point','Response','Control Output','Pitch')
    xlabel('Time (sec)')
    ylabel('Amplitude (m/s, deg)')
    title('Position Rate Controller - Vx')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    stairs(v_time,v_data(1:v_sizes(1),[3 5 6 8]))
    legend('Set Point','Response','Control Output','Roll')
    xlabel('Time (sec)')
    ylabel('Amplitude (m/s, deg)')
    title('Position Rate Controller - Vy')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(v_time,v_data(1:v_sizes(1),10))
    plot(v_time, ones(1,v_sizes(1)).*mean(v_data(1:v_sizes(1),10)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Position Controller Time Performance - Process')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(v_time,v_data(1:v_sizes(1),11))
    plot(v_time, ones(1,v_sizes(1)).*mean(v_data(1:v_sizes(1),11)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Position Controller Time Performance - Period')
    hold off
    
    figure()
    subplot(2,2,1)
    hold on
    grid on
    plot(p_time, p_data(1:p_sizes(1), 6), 'LineWidth', 1.5)
    plot(p_time, p_data(1:p_sizes(1),[10 11 12]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Controller Separate Output - X')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    plot(p_time, p_data(1:p_sizes(1), 7), 'LineWidth', 1.5)
    plot(p_time, p_data(1:p_sizes(1),[13 14 15]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Controller Separate Output - Y')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(v_time, v_data(1:v_sizes(1), 7), 'LineWidth', 1.5)
    plot(v_time, v_data(1:v_sizes(1),[12 13 14]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Rate Controller Separate Output - Vx')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(v_time, v_data(1:v_sizes(1), 6), 'LineWidth', 1.5)
    plot(v_time, v_data(1:v_sizes(1),[15 16 17]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Rate Controller Separate Output - Vy')
    hold off
    
end

%% Heading Control Plot
function hdg_ctrl_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Heading Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Heading Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure()
    subplot(2,2,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),[2 3 5]))
    legend('Set Point','Response','Control Output')
    xlabel('Time (sec)')
    ylabel('Amplitude (deg, deg/s)')
    title('Heading Controller')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    stairs(time,data(1:sizes(1),4))
    legend('Heading')
    xlabel('Time (sec)')
    ylabel('Amplitude (deg)')
    title('Heading Controller - Heading')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(time,data(1:sizes(1),6))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),6)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Heading Controller Time Performance - Process')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(time,data(1:sizes(1),7))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),7)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Heading Controller Time Performance - Period')
    hold off
    
    figure()
    hold on
    grid on
    plot(time, data(1:sizes(1), 5), 'LineWidth', 1.5)
    plot(time, data(1:sizes(1),[8 9 10]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Heading Controller Separate Output')
    hold off

end

%% Command Transmit Plot
function cmd_trans_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Command Transmit.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Command Transmit.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure()
    subplot(2,2,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),[2 3 5]))
    legend('Roll','Pitch','Thrust')
    xlabel('Time (sec)')
    ylabel('Amplitude (deg, Thrust Scale)')
    title('Command Transmit')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    stairs(time,data(1:sizes(1),4))
    legend('Yaw Rate')
    xlabel('Time (sec)')
    ylabel('Amplitude (deg/s)')
    title('Command Transmit - Yaw Rate')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(time,data(1:sizes(1),6))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),6)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Command Transmit Time Performance - Process')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(time,data(1:sizes(1),7))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),7)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Command Transmit Time Performance - Period')
    hold off

end

%% 3D Plot
function position_state_plot(file_time_stamp)
    xy_data = readmatrix(file_time_stamp+"_Position Controller.csv");
    z_data = readmatrix(file_time_stamp+"_Altitude Controller.csv");
    
    figure()
    plot3(-xy_data(:,5),-xy_data(:,4),z_data(:,3),[0.21 -0.21 -0.21 0.21 0.21],[0.21 0.21 -0.21 -0.21 0.21],[0 0 0 0 0],'--')
    grid on
    legend('Path','Carpet Boundary')
    xlabel('Front-Back (m)')
    ylabel('Left-Right (m)')
    zlabel('Altitude (m)')
    title('Flight Path (Global Frame) - From Vision and LiDAR')
end
