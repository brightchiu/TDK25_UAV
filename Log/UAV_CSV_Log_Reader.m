% Read CSV File to demostrate plot
% For PGM Log Analysis
% PO-JUI, CHIU    2021/12/12

clc; clear; close all;

%% File Designate
% Prefix file name -> time stamp
file_time_stamp = "12-12 20-46-59";
file_name_list = [" Controller"; "Altitude Velocity Controller"; "Altitude Sensor";
    "Altitude Sensor - Lidar"; "Position Controller"; "Velocity Controller"; "Heading Controller";
    "Camera Sensor"; "Command Transfer"]';

%% Load data
alt_ctrl_plot(file_time_stamp)
alt_sens_plot(file_time_stamp)
alt_sens_lidar_plot(file_time_stamp)
cam_sens_plot(file_time_stamp)
pos_ctrl_plot(file_time_stamp)
hdg_ctrl_plot(file_time_stamp)
cmd_trans_plot(file_time_stamp)
position_state_plot_2d(file_time_stamp)
position_state_plot(file_time_stamp)
%}
%% Altitude Control Plot
function alt_ctrl_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Altitude Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure('Name','Altitude Controller','Position',[0 0 1200 675])
    subplot(2,2,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),2:4))
    legend('Set Point','Response','Control Output')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Altitude Controller')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(time,data(1:sizes(1),6))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),6)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Altitude Controller Time Performance - Process')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(time,data(1:sizes(1),7))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),7)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Altitude Controller Time Performance - Period')
    hold off
    
    head = readmatrix(file_time_stamp+"_Altitude Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    subplot(2,2,2)
    hold on
    grid on
    plot(time, data(1:sizes(1), 4), 'LineWidth', 1.5)
    plot(time, data(1:sizes(1),8:10))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Altitude Controller Separate Output')
    hold off

end

%% Altitude Sensor Plot
function alt_sens_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Altitude Sensor.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Altitude Sensor.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure('Name','Altitude Sensor','Position',[0 0 1200 675])
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
    
    figure('Name','Altitude Sensor - Lidar','Position',[0 0 1200 675])
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
    
    figure('Name','Camera Sensor','Position',[0 0 1200 675])
    subplot(2,3,1)
    hold on
    grid on
    stairs(time,data(1:sizes(1),[4 6]))
    legend('X','Vx')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Camera Sensor - X State')
    hold off
    
    subplot(2,3,4)
    hold on
    grid on
    stairs(time,data(1:sizes(1),[5 7]))
    legend('Y','Vy')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Camera Sensor - Y State')
    hold off
    
    subplot(2,3,2)
    hold on
    grid on
    stairs(time,data(1:sizes(1),8))
    legend('Angle')
    xlabel('Time (sec)')
    ylabel('Amplitude (degree)')
    title('Camera Sensor - Angle State')
    hold off
    
    subplot(2,3,5)   
    hold on
    grid on
    stairs(time,data(1:sizes(1),[9 12]))
    legend('Area', 'Line Area')
    xlabel('Time (sec)')
    ylabel('Amplitude (m^2)')
    title('Camera Sensor - Area State')
    hold off
    
    subplot(2,3,3)
    hold on
    grid on
    plot(time,data(1:sizes(1),10))
    plot(time, ones(1,sizes(1)).*mean(data(1:sizes(1),10)),'-','LineWidth',1.5)
    legend('Measurement','Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Camera Sensor Time Performance - Measurement')
    hold off
    
    subplot(2,3,6)
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
    
    figure('Name','Position Controller','Position',[0 0 1200 675])
    subplot(2,2,1)
    hold on
    grid on
    stairs(p_time,p_data(1:p_sizes(1),[2 4 7]))
    legend('Set Point','Response','Pitchsp')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Position Controller - X')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    stairs(p_time,p_data(1:p_sizes(1),[3 5 6]))
    legend('Set Point','Response','Rollsp')
    xlabel('Time (sec)')
    ylabel('Amplitude (m, m/s)')
    title('Position Controller - Y')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(p_time,p_data(1:p_sizes(1),12))
    plot(p_time, ones(1,p_sizes(1)).*mean(p_data(1:p_sizes(1),12)),'-','LineWidth',1.5)
    legend('Process', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Position Controller Time Performance - Process')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(p_time,p_data(1:p_sizes(1),13))
    plot(p_time, ones(1,p_sizes(1)).*mean(p_data(1:p_sizes(1),13)),'-','LineWidth',1.5)
    legend('Period', 'Mean')
    xlabel('Time (sec)')
    ylabel('Duration (sec)')
    title('Position Controller Time Performance - Period')
    hold off
    
    figure('Name','Position Controller Separate Output','Position',[0 0 1200 675])
    subplot(2,2,1)
    hold on
    grid on
    plot(p_time, p_data(1:p_sizes(1), 7), 'LineWidth', 1.5)
    plot(p_time, p_data(1:p_sizes(1),[14 15 16]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Controller Separate Output - X')
    hold off
    
    subplot(2,2,2)
    hold on
    grid on
    plot(p_time, p_data(1:p_sizes(1), 6), 'LineWidth', 1.5)
    plot(p_time, p_data(1:p_sizes(1),[17 18 19]))
    legend('Control Output', 'K_p Output', 'K_i Output', 'K_d Output')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Controller Separate Output - Y')
    hold off
    
    subplot(2,2,3)
    hold on
    grid on
    plot(p_time, p_data(1:p_sizes(1),[6 8 10]))
    legend('Roll_s_p', 'Roll_u_a_v', 'Roll_B_N_O_0_5_5')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Controller - Roll')
    hold off
    
    subplot(2,2,4)
    hold on
    grid on
    plot(p_time, p_data(1:p_sizes(1),[7 9 11]))
    legend('Pitch_s_p', 'Pitch_u_a_v', 'Pitch_B_N_O_0_5_5')
    xlabel('Time (sec)')
    ylabel('Amplitude')
    title('Position Controller - Pitch')
    hold off
end

%% Heading Control Plot
function hdg_ctrl_plot(file_time_stamp)
    head = readmatrix(file_time_stamp+"_Heading Controller.csv",'Range','1:1','OutputType','string');
    data = readmatrix(file_time_stamp+"_Heading Controller.csv");
    time = data(:,1);
    sizes = size(data);
    
    figure('Name','Heading Controller','Position',[0 0 1200 675])
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
    
    figure('Name','Heading Controller Separate Output','Position',[0 0 800 450])
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
    
    figure('Name','Command Transmit','Position',[0 0 1200 675])
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

%% 2D Plot
function position_state_plot_2d(file_time_stamp)
    xy_data = readmatrix(file_time_stamp+"_Position Controller.csv");
    
    figure('Name','2D Flight Path','Position',[0 0 1200 675])
    plot(xy_data(:,5),xy_data(:,4),[0.21 -0.21 -0.21 0.21 0.21],[0.21 0.21 -0.21 -0.21 0.21],'--')
    grid on
    axis equal
    xlim([-1 1])
    ylim([-1 1])
    legend('Path','Carpet Boundary')
    xlabel('Left-Right-Y (m)')
    ylabel('Front-Back-X (m)')
    zlabel('Altitude (m)')
    title('Flight Path (Global Frame) - From Vision and LiDAR')
end

%% 3D Plot
function position_state_plot(file_time_stamp)
    xy_data = readmatrix(file_time_stamp+"_Position Controller.csv");
    z_data = readmatrix(file_time_stamp+"_Altitude Controller.csv");
    
    figure('Name','3D Flight Path','Position',[0 0 1200 675])
    plot3(xy_data(:,5),xy_data(:,4),z_data(:,3),[0.21 -0.21 -0.21 0.21 0.21],[0.21 0.21 -0.21 -0.21 0.21],[0 0 0 0 0],'--')
    grid on
    axis equal
    xlim([-1 1])
    ylim([-1 1])
    zlim([0 2])
    legend('Path','Carpet Boundary')
    xlabel('Left-Right (m)')
    ylabel('Front-Back (m)')
    zlabel('Altitude (m)')
    title('3D Flight Path (Global Frame) - From Vision and LiDAR')
end
