%% Load GPS data from flight log file
% format: GPS (now the measurement frequency is 5Hz)
clear;
addpath(['..' filesep '..' filesep 'common']);

%% define the log file's path
%log_filename='2020-11-09 10-36-06.txt';
%log_filename='2020-11-10 16-58-58.txt';
log_filename='2020-11-10 17-09-50.txt';

%log_file_path=['..' filesep '..' filesep 'dataset' filesep 'indoor_dataset_2' filesep log_filename];
%log_file_path=['..' filesep '..' filesep 'dataset' filesep 'outdoor_dataset_0' filesep log_filename];
%log_file_path=['..' filesep '..' filesep 'dataset' filesep 'outdoor_dataset_1' filesep log_filename];
log_file_path=['..' filesep '..' filesep 'dataset' filesep 'outdoor_dataset_2' filesep log_filename];

fid=fopen(log_file_path);
raw_data = textscan(fid,'%s'); % solve the problem for some log files which are segmented with an extra space or tab
raw_data = raw_data{1,1};
fclose(fid); 

%% record data

gps_measure_count=0;

time_us=[]; % (unit: us)
gps_week=[];
gps_sec_in_week=[]; % (unit: ms)
status=[];
sat_num=[];
hdop=[];
lat=[]; % latitude in degree
lon=[]; % longitude in degree
alt=[]; % altitude (height) in m
spd=[];
gps_unix_gap_us=[];

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
       gps_sec_in_week = [gps_sec_in_week; cur_str_split(4)];
       gps_week = [gps_week; cur_str_split(5)];
       
       sat_num=[sat_num; cur_str_split(6)];
       hdop=[hdop; cur_str_split(7)];
       lat=[lat; cur_str_split(8)];
       lon=[lon; cur_str_split(9)];
       alt=[alt; cur_str_split(10)];
       spd=[spd; cur_str_split(11)];
       
       gps_unix_gap_us_temp=gps_sec_in_week*1e3-time_us; % GPST - UNIXT(us)
       gps_unix_gap_us=[gps_unix_gap_us; gps_unix_gap_us_temp]; 
       
       gps_measure_count=gps_measure_count+1;
   end
end

disp(['Collect [', num2str(gps_measure_count), '] GPS data.']); 

gps_unix_gap_us_mean=mean(gps_unix_gap_us); % GPST - UNIXT(us)
gps_unix_gap_us_std=std(gps_unix_gap_us);
fprintf('[GPS Time] - [UNIX TIME] = %8.3f ( s )\t+-%5.2f ( ms )\n',gps_unix_gap_us_mean*1e-6, gps_unix_gap_us_std*1e-3);

%% Convert to local ENU coordinate system
[e,n,u]=blh2enu(lat,lon,alt,lat(1),lon(1),alt(1));

%% Plot
figure(1);
plottraj([e,n,u]);
grid on;
axis equal;
xlabel('E(m)');
ylabel('N(m)');
zlabel('U(m)');
legend('Start','End');
title('GPS measurements in Local ENU');

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

