%% Load Attitude data from flight log file
% format: ATT,time_boot_ms,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed 
%clear; clc; close all;
addpath(['..' filesep '..' filesep 'common']);

%% define the log file's path
%log_filename='2020-11-09 10-36-06.txt'; % outdoor0
%log_filename='2020-11-10 16-58-58.txt'; % outdoor1
%log_filename='2020-11-10 17-09-50.txt'; % outdoor2
%log_filename='2020-11-12 16-51-33.txt'; % outdoor3
%log_filename='2020-11-12 17-18-38.txt';  % outdoor4
log_filename='2020-11-15 16-37-07.txt';  % outdoor5

%log_file_path=['..' filesep '..' filesep 'dataset' filesep 'indoor_dataset_2' filesep log_filename];
log_file_path=['..' filesep '..' filesep 'dataset' filesep 'outdoor_dataset_5' filesep log_filename];

fid=fopen(log_file_path);
raw_data = textscan(fid,'%s'); % solve the problem for some log files which are segmented with an extra space or tab
raw_data = raw_data{1,1};
fclose(fid); 

%% record data
attitude_measure_count=0;

time_us=[];
roll=[];
pitch=[];
yaw=[];
des_roll=[];
des_pitch=[];
des_yaw=[];
err_rp=[];
err_yaw=[];

for i=1:size(raw_data,1)
   current_str = raw_data{i};
   len_str = size(current_str,2);
   if (len_str < 3)
       continue; 
   end
   current_str_head = current_str(1:3);
   
   % Read attitude data
   % time_boot_ms roll pitch yaw rollspeed pitchspeed yawspeed
   if(current_str_head == 'ATT')
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       time_us = [time_us; cur_str_split(2)];
       des_roll=[des_roll; cur_str_split(3)];
       roll=[roll; cur_str_split(4)];
       des_pitch=[des_pitch; cur_str_split(5)];
       pitch=[pitch; cur_str_split(6)];
       des_yaw=[des_yaw; cur_str_split(7)];
       yaw=[yaw; cur_str_split(8)];
       err_rp=[err_rp; cur_str_split(9)];
       err_yaw=[err_yaw; cur_str_split(10)];
       
       attitude_measure_count=attitude_measure_count+1;
   end
end

disp(['Collect [', num2str(attitude_measure_count), '] attitude data.']); 

%% Plot
figure(1);
plot(1e-6*time_us, [roll pitch yaw]);
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
legend('roll','pitch','yaw', 'Fontname', 'Times New Roman','FontSize',10);
xlabel('timestamp (s)', 'Fontname', 'Times New Roman','FontSize',14);
ylabel('deg', 'Fontname', 'Times New Roman','FontSize',14);
title('Attitude','Fontname', 'Times New Roman','FontSize',16);

figure(2);
plot(1e-6*time_us, [des_roll des_pitch des_yaw]);
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
legend('des roll','des pitch','des yaw', 'Fontname', 'Times New Roman','FontSize',10)
xlabel('timestamp (s)', 'Fontname', 'Times New Roman','FontSize',14);
ylabel('deg', 'Fontname', 'Times New Roman','FontSize',14);
title('Attitude Des', 'Fontname', 'Times New Roman','FontSize',16);

