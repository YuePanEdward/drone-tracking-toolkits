%% Load GPS data from flight log file
% format: GPS (now the measurement frequency is 5Hz)
% clear; clc; close all;
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

%% Set the begin and end time 
%for test 5 
%begin_gps_sow = 57069.0;
%end_gps_sow =  57118.0;
begin_gps_sow = 0;
end_gps_sow = inf;
% begin_gps_sow = meas_gps_sow_tracked(1);
% end_gps_sow = meas_gps_sow_tracked(end);

%% record data
gps_measure_count=0;

time_us=[]; % (unit: us)
gps_week=[];
time_gps_sow=[]; % gps second in week (unit: s)
status=[];
sat_num=[];
hdop=[];
lat=[]; % latitude in degree
lon=[]; % longitude in degree
alt=[]; % altitude (height) in m
spd=[];
gps_unix_gap=[];  % (unit: s)

for i=1:size(raw_data,1)
   current_str = raw_data{i};
   len_str = size(current_str,2);
   if (len_str < 3)
       continue; 
   end
   current_str_head = current_str(1:3);
   
   if(current_str_head == 'GPS')
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       time_us = [time_us; cur_str_split(2)];
       status = [status; cur_str_split(3)];
       time_gps_sow = [time_gps_sow; cur_str_split(4)*1e-3];  % unit: ms --> s
       gps_week = [gps_week; cur_str_split(5)];
       
       sat_num=[sat_num; cur_str_split(6)];
       hdop=[hdop; cur_str_split(7)];
       lat=[lat; cur_str_split(8)];
       lon=[lon; cur_str_split(9)];
       alt=[alt; cur_str_split(10)];
       spd=[spd; cur_str_split(11)];
       
       gps_unix_gap_temp=cur_str_split(4)*1e-3-cur_str_split(2)*1e-6; % GPST - DroneT, unit: s
       gps_unix_gap=[gps_unix_gap; gps_unix_gap_temp];  %  unit: s
       
       gps_measure_count=gps_measure_count+1;
   end
end

disp(['Collect [', num2str(gps_measure_count), '] GPS data.']); 

gps_unix_gap_mean=mean(gps_unix_gap);  % GPST - DroneT, unit: s
gps_unix_gap_std=std(gps_unix_gap);         % unit: s
fprintf('[GPS Time] - [UNIX TIME] = %8.3f ( s )\t+-%5.2f ( ms )\n',gps_unix_gap_mean, gps_unix_gap_std*1e3);

%% Convert to local ENU coordinate system
[e,n,u]=blh2enu(lat,lon,alt,lat(1),lon(1),alt(1));
points3d_enu=[e,n,u];

%% Get the interested part (tracked part)
track_index = (time_gps_sow  >=  begin_gps_sow & time_gps_sow  <=  end_gps_sow); % unit: s
points3d_enu = points3d_enu(track_index,:);

%% Plot
figure(1);
plottraj(points3d_enu);
grid on;
axis equal;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
xlabel('E(m)','Fontname', 'Times New Roman','FontSize',14);
ylabel('N(m)','Fontname', 'Times New Roman','FontSize',14);
zlabel('U(m)','Fontname', 'Times New Roman','FontSize',14);
legend('Start','End','Fontname', 'Times New Roman','FontSize',10);
title('GPS measurements in Local ENU','Fontname', 'Times New Roman','FontSize',16);

% figure(2);
% plot(1e-3*time_us, [lat lon alt]);
% grid on;
% legend('latitude','longitude','altitude');
% xlabel('timestamp (ms)');
% ylabel('deg or m');
% title('GPS measurements in WGS84');
% 
% figure(3);
% plot(1e-3*time_us, sat_num);
% grid on;
% xlabel('timestamp (ms)');
% ylabel('number'); 
% title('Satellite number');

