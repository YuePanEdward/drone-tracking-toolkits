%% Load IMU data from flight log file
% format: IMU,QffffffIIfBBHH,TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz
%clear; clc; close all;
addpath(['..' filesep '..' filesep 'common']);

%% select from three different IMU names (IMU, IMU2, IMU3)
% IMUname='IMU,';
IMUname='IMU2';
% IMUname='IMU3';

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

%% Set the begin and end time 
%test
begin_gps_sow = 57069.0; % (unit: s)
end_gps_sow =  57118.0;
% begin_gps_sow = 0;
% end_gps_sow = inf;

%% record data
imu_measure_count=0;

time_us=[];
xgyro=[];
ygyro=[];
zgyro=[];
xacc=[];
yacc=[];
zacc=[];
eg=[];
ea=[];
t=[];
gh=[];
ah=[];
ghz=[];
ahz=[];

for i=1:size(raw_data,1)
   current_str = raw_data{i};
   len_str = size(current_str,2);
   if (len_str < 4)
       continue; 
   end
   current_str_head = current_str(1:4);
   if(current_str_head == IMUname)
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       time_us = [time_us; cur_str_split(2)];
       
       % convert to gps_time
       xgyro=[xgyro; cur_str_split(3)];
       ygyro=[ygyro; cur_str_split(4)];
       zgyro=[zgyro; cur_str_split(5)];
       xacc=[xacc; cur_str_split(6)];
       yacc=[yacc; cur_str_split(7)];
       zacc=[zacc; cur_str_split(8)];
       eg = [eg; cur_str_split(9)];
       ea = [ea; cur_str_split(10)];
       t = [t; cur_str_split(11)];
       gh = [gh; cur_str_split(12)];
       ah = [ah; cur_str_split(13)];
       ghz = [ghz; cur_str_split(14)];
       ahz = [ahz; cur_str_split(15)];
       
       imu_measure_count=imu_measure_count+1;
   end
end

disp(['Collect [', num2str(imu_measure_count), '] IMU data.']); 

%% Plot
figure(1);
plot(1e-6*time_us, [xacc yacc zacc]);
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
legend('ax','ay','az','Fontname', 'Times New Roman','FontSize',10);
xlabel('timestamp (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('acceleration (0.01 m/s^2)','Fontname', 'Times New Roman','FontSize',14);
grid on;
title('IMU accelerometer measurements','Fontname', 'Times New Roman','FontSize',16);

figure(2);
plot(1e-6*time_us, 180.0/pi*[xgyro ygyro zgyro]);
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
legend('wx','wy','wz','Fontname', 'Times New Roman','FontSize',10);
xlabel('timestamp (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('angular velocity (deg/s)','Fontname', 'Times New Roman','FontSize',14); 
grid on;
title('IMU gyroscope measurements','Fontname', 'Times New Roman','FontSize',16);
