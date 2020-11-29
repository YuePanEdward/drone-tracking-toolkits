clear; clc; close all;
format long g;
addpath(['..' filesep '..' filesep 'common']);
mkdir ('results');

%% Log data path
% log_filename='2020-11-15 16-37-07';  % outdoor5
% log_file_path=['..' filesep '..' filesep 'dataset' filesep 'outdoor_dataset_5'  filesep log_filename '.txt'];

% log_filename='2020-11-23 17-30-31';  % outdoor7
% log_file_path=['..' filesep '..' filesep 'dataset' filesep 'outdoor_dataset_7'  filesep log_filename '.txt'];

log_filename='2020-11-25 17-21-24';  % outdoor8
log_file_path=['..' filesep '..' filesep 'dataset' filesep 'outdoor_dataset_8'  filesep log_filename '.txt'];

mkdir ('results',  log_filename); % create subfolder

fid=fopen(log_file_path);
raw_data = textscan(fid,'%s'); % solve the problem for some log files which are segmented with an extra space or tab
raw_data = raw_data{1,1};
fclose(fid); 

%% Set the begin and end time 
% for test 5 
% begin_gps_sow = 57069.0;
% end_gps_sow =  57118.0;
% for test 7 
% begin_gps_sow = 146010.0;
% end_gps_sow =  146095.0;
% for test 8 
begin_gps_sow = 318110.0;
end_gps_sow =  318210.0;
% begin_gps_sow = 0;
% end_gps_sow = inf;
% begin_gps_sow = meas_gps_sow_tracked(1);
% end_gps_sow = meas_gps_sow_tracked(end);

%% record GPS data
gps_measure_count=0;

time_us_gps=[]; % drone time (unit: us)
gps_week=[];
time_gps_sow_gps=[]; % gps second in week of the gps (unit: s) 
status=[];  % 1: no gps , 3: 3d fixed, 4: fine GPS condition (with SBAS, not with RTK)
sat_num=[]; % available GNSS satellite number
hdop=[]; % horizontal dilution of precision
lat=[]; % latitude in degree
lon=[]; % longitude in degree
alt=[]; % altitude (height) in m
spd=[];

vdop=[]; % vertical dilution of precision
hacc=[];  % horizontal accuracy (m)
vacc=[];  % vertical accuracy (m)
gps_drone_gap=[];  % GPST - DroneT, unit: s

for i=1:size(raw_data,1)
   current_str = raw_data{i};
   len_str = size(current_str,2);
   if (len_str < 3)
       continue; 
   end
   current_str_head = current_str(1:3);
   % Get GPS measurement data
   if(current_str_head == 'GPS')
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       if (isnan(cur_str_split) | cur_str_split(3)<3) % data is not complete or GPS is not available
            continue; 
       end
       time_us_gps = [time_us_gps; cur_str_split(2)];
       status = [status; cur_str_split(3)];
       time_gps_sow_gps = [time_gps_sow_gps; cur_str_split(4)*1e-3];  % unit: ms --> s
       gps_week = [gps_week; cur_str_split(5)];
       
       sat_num=[sat_num; cur_str_split(6)];
       hdop=[hdop; cur_str_split(7)];
       lat=[lat; cur_str_split(8)];
       lon=[lon; cur_str_split(9)];
       alt=[alt; cur_str_split(10)];
       spd=[spd; cur_str_split(11)];
       
       gps_drone_gap_temp=cur_str_split(4)*1e-3- cur_str_split(2)*1e-6; % GPST - DroneT, unit: s
       gps_drone_gap=[gps_drone_gap; gps_drone_gap_temp];  %  unit: s
       
       gps_measure_count=gps_measure_count+1;
   end
   % Get GPS accuracy information
   if(current_str_head == 'GPA')
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       vdop = [vdop; cur_str_split(3)];
       hacc=[hacc; cur_str_split(4)];
       vacc=[vacc; cur_str_split(5)];
   end
end
disp(['Collect [', num2str(gps_measure_count), '] GPS data.']); 

gps_drone_gap_mean=mean(gps_drone_gap);  % GPST - DroneT, unit: s
gps_drone_gap_std=std(gps_drone_gap);         % unit: s
fprintf('[GPS Time] - [UNIX TIME] = %8.3f ( s )\t+-%5.2f ( ms )\n', gps_drone_gap_mean, gps_drone_gap_std*1e3);

%% Convert to local ENU coordinate system
[e,n,u]=blh2enu(lat,lon,alt,lat(1),lon(1),alt(1));
points3d_enu_gps=[e,n,u];

% store the referenced wgs84 coordinate (lat, lon, alt) of the begining point
ref_wgs =[lat(1),lon(1),alt(1)];
save(['results' filesep log_filename filesep log_filename '_ref_wgs.mat'],'ref_wgs');
disp('Save reference coordinate done');  %act as the origin of the local reference coordinte system

%% Get the interested part (tracked part)
track_index_gps = (time_gps_sow_gps  >=  begin_gps_sow & time_gps_sow_gps  <=  end_gps_sow); % unit: s
track_points3d_enu_gps = points3d_enu_gps(track_index_gps,:);
track_time_gps_sow_gps = time_gps_sow_gps(track_index_gps,:);
hacc = hacc(track_index_gps,:);
vacc = vacc(track_index_gps,:);

%% Load IMU data

% select from three different IMU names (IMU, IMU2, IMU3)
% IMU (IMU1): MPU6000
% IMU2: LDG20H + LSM303D

IMUname='IMU,'; 
%IMUname='IMU2';
% IMUname='IMU3';

%% record IMU data (use the default LOG mask setting)
imu_measure_count=0;

time_us_imu=[];
time_gps_sow_imu = [];
xgyro=[];
ygyro=[];
zgyro=[];
xacce=[];
yacce=[];
zacce=[];
eg=[];
ea=[];
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
       if (isnan(cur_str_split))
               continue; 
       end
       time_us_imu = [time_us_imu; cur_str_split(2)]; % unit: us
        % convert to gps_time
       temp_time_gps_sow_imu = cur_str_split(2)*1e-6 + gps_drone_gap_mean; % unit: s
       time_gps_sow_imu = [time_gps_sow_imu; temp_time_gps_sow_imu]; % unit: s
       
       % gyroscope
       xgyro=[xgyro; cur_str_split(3)];
       ygyro=[ygyro; cur_str_split(4)];
       zgyro=[zgyro; cur_str_split(5)];
       % accelerometer
       xacce=[xacce; cur_str_split(6)];
       yacce=[yacce; cur_str_split(7)];
       zacce=[zacce; cur_str_split(8)];
       % others
       eg = [eg; cur_str_split(9)];
       ea = [ea; cur_str_split(10)];
       gh = [gh; cur_str_split(12)];
       ah = [ah; cur_str_split(13)];
       ghz = [ghz; cur_str_split(14)];
       ahz = [ahz; cur_str_split(15)];
       
       imu_measure_count=imu_measure_count+1;
   end
end

disp(['Collect [', num2str(imu_measure_count), '] IMU data.']); 

%% Get the interested part (tracked part)
track_index_imu = (time_gps_sow_imu  >=  begin_gps_sow & time_gps_sow_imu  <=  end_gps_sow); % unit: s
track_time_gps_sow_imu = time_gps_sow_imu(track_index_imu,:);
track_time_gps_sow_acce = track_time_gps_sow_imu;
track_time_gps_sow_gyro = track_time_gps_sow_imu;
track_acce = [xacce(track_index_imu,:)   yacce(track_index_imu,:)   zacce(track_index_imu,:)];
track_gyro = [xgyro(track_index_imu,:)   ygyro(track_index_imu,:)   zgyro(track_index_imu,:)];

%% Load the high frequency IMU data respectively (this is for the raw LOG mask setting)
% Accename='ACC1';
% % Accename='ACC2';
% % Accename='ACC3';
% Gyroname='GYR1'; 
% % Gyroname='GYR2'; 
% % Gyroname='GYR3'; 
% 
% %% record IMU data 
% acce_measure_count=0;
% gyro_measure_count=0;
% 
% time_us_acce=[];
% time_gps_sow_acce = [];
% time_us_gyro=[];
% time_gps_sow_gyro = [];
% xgyro=[];
% ygyro=[];
% zgyro=[];
% xacce=[];
% yacce=[];
% zacce=[];
% 
% for i=1:size(raw_data,1)
%    current_str = raw_data{i};
%    len_str = size(current_str,2);
%    if (len_str < 4)
%        continue; 
%    end
%    current_str_head = current_str(1:4);
%    if(current_str_head == Accename)
%        cur_str_split = split(current_str,',');
%        cur_str_split = str2double(cur_str_split);
%        if (isnan(cur_str_split))
%             continue; 
%        end
%        time_us_acce = [time_us_acce; cur_str_split(2)]; % unit: us
%         % convert to gps_time
%        temp_time_gps_sow_acce = cur_str_split(2)*1e-6 + gps_drone_gap_mean; % unit: s
%        time_gps_sow_acce = [time_gps_sow_acce; temp_time_gps_sow_acce]; % unit: s
%        
%        % accelerometer
%        xacce=[xacce; cur_str_split(4)];
%        yacce=[yacce; cur_str_split(5)];
%        zacce=[zacce; cur_str_split(6)];
%    
%        acce_measure_count=acce_measure_count+1;
%    end
% end
% 
% disp(['Collect [', num2str(acce_measure_count), '] Acceleration data.']); 
% 
% for i=1:size(raw_data,1)
%    current_str = raw_data{i};
%    len_str = size(current_str,2);
%    if (len_str < 4)
%        continue; 
%    end
%    current_str_head = current_str(1:4);
%    if(current_str_head == Gyroname)
%        cur_str_split = split(current_str,',');
%        cur_str_split = str2double(cur_str_split);
%        if (isnan(cur_str_split))
%             continue; 
%        end
%        time_us_gyro= [time_us_gyro; cur_str_split(2)]; % unit: us
%         % convert to gps_time
%        temp_time_gps_sow_gyro = cur_str_split(2)*1e-6 + gps_drone_gap_mean; % unit: s
%        time_gps_sow_gyro = [time_gps_sow_gyro; temp_time_gps_sow_gyro]; % unit: s
%        
%        % accelerometer
%        xgyro=[xgyro; cur_str_split(4)];
%        ygyro=[ygyro; cur_str_split(5)];
%        zgyro=[zgyro; cur_str_split(6)];
%    
%        gyro_measure_count=gyro_measure_count+1;
%    end
% end
% 
% disp(['Collect [', num2str(gyro_measure_count), '] Gyroscope data.']); 
% 
% %% Get the interested part (tracked part)
% track_index_acce = (time_gps_sow_acce  >=  begin_gps_sow & time_gps_sow_acce <=  end_gps_sow); % unit: s
% track_index_gyro = (time_gps_sow_gyro  >=  begin_gps_sow & time_gps_sow_gyro <=  end_gps_sow); % unit: s
% track_time_gps_sow_acce = time_gps_sow_acce(track_index_acce,:);
% track_time_gps_sow_gyro = time_gps_sow_gyro(track_index_gyro,:);
% track_acce = [xacce(track_index_acce,:)   yacce(track_index_acce,:)   zacce(track_index_acce,:)];
% track_gyro = [xgyro(track_index_gyro,:)   ygyro(track_index_gyro,:)   zgyro(track_index_gyro,:)];

%% Load the fused data (EKF)

%% record NKF 1 data
% NKF 1 & NKF 6 (different IMU)
% * TimeUS - time stamp (uSec)
% * Roll,Pitch,Yaw - Euler roll, pitch and yaw angle (deg)
% * VN,VE - Velocity North,East direction (m/s)
% * VD, dPD - Velocity and Position Derivative Down (m/s)
% * PN,PE,PD - Position North,East,Down (m)
% * GX,GY,GZ - X,Y,Z rate gyro bias (deg/sec)
ekf_count=0;

time_us_ekf=[];
time_gps_sow_ekf = [];
roll_ekf=[];  % degree
pitch_ekf=[];  % degree
yaw_ekf=[]; % degree
pn_ekf=[]; % north position (m)
pe_ekf=[]; % east position (m)
pd_ekf=[]; % down position (m)

position_std_ekf=[]; % postioning standard deviation (m)

for i=1:size(raw_data,1)
   current_str = raw_data{i};
   len_str = size(current_str,2);
   if (len_str < 4)
       continue; 
   end
   current_str_head = current_str(1:4);
   if(current_str_head == 'NKF1')
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       if (isnan(cur_str_split))
            continue; 
       end
       time_us_ekf = [time_us_ekf; cur_str_split(2)]; % unit: us
        % convert to gps_time
       temp_time_gps_sow_ekf = cur_str_split(2)*1e-6 + gps_drone_gap_mean; % unit: s
       time_gps_sow_ekf = [time_gps_sow_ekf; temp_time_gps_sow_ekf]; % unit: s
       
       % attitude
       roll_ekf=[roll_ekf; cur_str_split(3)];
       pitch_ekf=[pitch_ekf; cur_str_split(4)];
       yaw_ekf=[yaw_ekf; cur_str_split(5)];
       % position
       pn_ekf=[pn_ekf; cur_str_split(10)];
       pe_ekf=[pe_ekf; cur_str_split(11)];
       pd_ekf=[pd_ekf; cur_str_split(12)];
       % others
       
       ekf_count=ekf_count+1;
   end
   if(current_str_head == 'NKF4')
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       % position accuracy
       position_std_ekf = [position_std_ekf; cur_str_split(4)];
   end
end

disp(['Collect [', num2str(ekf_count), '] EKF fused data.']); 

%% Get the interested part (tracked part)
track_index_ekf = (time_gps_sow_ekf  >=  begin_gps_sow & time_gps_sow_ekf  <=  end_gps_sow); % unit: s
track_time_gps_sow_ekf = time_gps_sow_ekf(track_index_ekf,:);
track_attitude_ekf = [roll_ekf(track_index_ekf,:)   pitch_ekf(track_index_ekf,:)   yaw_ekf(track_index_ekf,:)]; % roll, pitch, yaw
track_position_ekf = [pe_ekf(track_index_ekf,:)   pn_ekf(track_index_ekf,:)   -pd_ekf(track_index_ekf,:)]; % e, n, u
track_position_std_ekf = position_std_ekf(track_index_ekf,:);

% save the fused pose and corresponding timestamp
save(['results' filesep log_filename filesep log_filename '_gpsts_ekf.mat'],'track_time_gps_sow_ekf');
save(['results' filesep log_filename filesep log_filename '_position_ekf.mat'],'track_position_ekf');
save(['results' filesep log_filename filesep log_filename '_attitude_ekf.mat'],'track_attitude_ekf');
disp('Save fused pose done');  

%%
% NKF 2
% * TimeUS - time stamp (uSec)
% * AZbias - Z accelerometer bias (cm/s/s)
% * GSX,GSY,GSZ - X,Y,Z rate gyro scale factor (%)
% * VWN,VWE - Wind velocity North,East (m/s)
% * MN,ME,MD - Earth magnetic field North,East,Down (mGauss)
% * MX,MY,MZ - Body fixed magnetic field X,Y,Z (mGauss)
% * MI - Index of the magnetometer being used by EKF2

% NKF 3 (filter innovation, difference of the measurement compared with the current state prediction)
% the larger the innovation is, the less accurate the IMU sensor is since
% it is used for prediction
% * TimeUS - time stamp (uSec)
% * IVN,IVE - GPS velocity innovations North, East (m/s)
% * IVD - GPS velocity innovation Down (m/s)
% * IPN,IPE - GPS position innovations North,East (m)
% * IPD - Barometer position innovation Down (m) for altitude determination
% * IMX,IMY,IMZ - Magnetometer innovations X,Y,Z (mGauss)
% * IYAW - Compass yaw innovation (deg)
% * IVT - True airspeed innovation (m/s)

% NKF 4 (Filter Health and Status), should be about 0.3 for a proper flight
% A value of less than 1 indicates that that measurement has passed its checks and is being used by the EKF2
% * TimeUS - time stamp (uSec)
% * SV - GPS velocity test ratio (Square root of the velocity variance)
% * SP - GPS position test ratio (Square root of the position variance),
% important
% * SH - Barometer test ratio  (Square root of the height variance)
% * SM - Magnetometer test ratio (Magnetic field variance)
% * SVT - Airspeed sensor Test ratio
% * OFN - Position jump North due to the last reset of the filter states (m)
% * OFE - Position jump East due to the last reset of the filter states (m)
% * FS - Integer bitmask of filter numerical faults
% * TS - Integer bitmask of filter measurement timeout
% * SS - Integer bitmask of filer solution status
% * GPS - Integer bitmask of filter GPS quality checks
% * PI - Index showing which instance of EKF2 has been selected for flight control

% NKF 5 (for optics flow and range finder, not used)

%% Plot
% trajectory measured by onboard GPS
figure(1);
plottraj(track_points3d_enu_gps);
grid on;
axis equal;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
xlabel('E(m)','Fontname', 'Times New Roman','FontSize',14);
ylabel('N(m)','Fontname', 'Times New Roman','FontSize',14);
zlabel('U(m)','Fontname', 'Times New Roman','FontSize',14);
legend('Start','End','Fontname', 'Times New Roman','FontSize',10);
title('GPS measurements in Local ENU','Fontname', 'Times New Roman','FontSize',16);

% EKF fused pose
figure(2);
status_vec= zeros(size(track_position_ekf,1),1);
plottraj(track_position_ekf, status_vec, track_attitude_ekf);
grid on;
axis equal;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
xlabel('E(m)','Fontname', 'Times New Roman','FontSize',14);
ylabel('N(m)','Fontname', 'Times New Roman','FontSize',14);
zlabel('U(m)','Fontname', 'Times New Roman','FontSize',14);
legend('Start','End','Fontname', 'Times New Roman','FontSize',10);
title('Fused pose in Local ENU','Fontname', 'Times New Roman','FontSize',16);

% GPS and fused accuracy
figure(3);
plot(track_time_gps_sow_gps, [hacc, vacc] ,'Linewidth', 2);
hold on;
plot(track_time_gps_sow_ekf, track_position_std_ekf ,'Linewidth', 2)
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
set(gca,'xticklabel',num2str((get(gca,'xtick'))'));  
legend('GNSS horizontal accuracy','GNSS vertical accuracy','GNSSINS fused accuracy', 'Fontname', 'Times New Roman','FontSize',10);
xlabel('timestamp (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('accuracy \sigma (m)','Fontname', 'Times New Roman','FontSize',14);
grid on;
title('Positioning accuracy','Fontname', 'Times New Roman','FontSize',16);

% acceration measured by onboard IMU
figure(4);
plot(track_time_gps_sow_acce, track_acce,'Linewidth', 2);
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
set(gca,'xticklabel',num2str((get(gca,'xtick'))'));  
legend('ax','ay','az','Fontname', 'Times New Roman','FontSize',10);
xlabel('timestamp (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('acceleration (0.01 m/s^2)','Fontname', 'Times New Roman','FontSize',14);
grid on;
title('IMU accelerometer measurements','Fontname', 'Times New Roman','FontSize',16);

% angular rate measured by onboard IMU
figure(5);
plot(track_time_gps_sow_gyro, 180.0/pi*track_gyro,'Linewidth', 2);
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
set(gca,'xticklabel',num2str((get(gca,'xtick'))'));  
legend('wx','wy','wz','Fontname', 'Times New Roman','FontSize',10);
xlabel('timestamp (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('angular velocity (deg/s)','Fontname', 'Times New Roman','FontSize',14); 
grid on;
title('IMU gyroscope measurements','Fontname', 'Times New Roman','FontSize',16);


