% Matlab codes for the automatic control of a Leica total station (TS50/60,
% TPS 1200, etc.)
% By YUE PAN @ ETHZ IGP 
% IPA project: Measuring Drone Trajectory using Total Stations with Visual Tracking

% TODO: add more comments, refine the code

%%
clear; clc; close all;
addpath(['..' filesep '..' filesep 'common']);
mkdir ('results');

%% Set GeoCOM port, dB (Baud) rate
% For TPS1200, set it in configure->interfaces setting->GSI/GeoCOM mode on
% For Nova TS 50/60, set it in (TODO)

% COM port number (check in the device manager)
% COMPort = '/dev/ttyUSB1';  %on Linux
COMPort = 'COM4';            %on Windows

% dB (Baud) rate
%dB = 115200;
dB = 19200;
%dB=9600;

%% Set PRSIM TYPE
% PRISM_ROUND = 0,
% PRISM_MINI = 1,
% PRISM_TAPE = 2,
% PRISM_360 = 3,     [used,large one]
% PRISM_USER1 = 4,
% PRISM_USER2 = 5,
% PRISM_USER3 = 6,
% PRISM_360_MINI = 7, [used,small one]
% PRISM_MINI_ZERO = 8,
% PRISM_USER = 9
% PRISM_NDS_TAPE = 10
prism_type = 7; 

%% Set TARGET TYPE
% REFLECTOR_TARGET = 0
% REFLECTORLESS_TARGET = 1
target_type = 0; 

%% Automatic Target Recognition (ATR) on or not
atr_state = 0; % ATR state on (1)

%%
% ANGLE MEASUREMENT TOLERANCE
% For Nova TS-50/60
% RANGE FROM 1[cc] ( =1.57079 E-06[ rad ], highest resolution, slowest) 
% TO 100[cc] ( =1.57079 E-04[ rad ], lowest resolution, fastest)
% For TPS-1200, the value can be a bit higher for faster performance
hz_tol = 1.57079e-04; % Horizontal tolerance (moderate resolution) 
v_tol = 1.57079e-04; % Vertical tolerance (moderate resolution)

%%
% DISTANCE MEASUREMENT MODE
% IR-Mode: Dist. Meas. With Reflector
% SINGLE_REF_STANDARD = 0,
% SINGLE_REF_FAST = 1,      [IR Fast]
% SINGLE_REF_VISIBLE = 2    [LO Standard]
% SINGLE_RLESS_VISIBLE = 3,
% CONT_REF_STANDARD = 4,    [used, IR Tracking]
% CONT_REF_FAST = 5,
% CONT_RLESS_VISIBLE = 6,
% AVG_REF_STANDARD = 7,
% AVG_REF_VISIBLE = 8,
% AVG_RLESS_VISIBLE = 9
% CONT_REF_SYNCHRO = 10;
% SINGLE_REF_PRECISE = 11;

distmode =4; % Default distance mode (from BAP_SetMeasPrg)
 
%% Open port, connect to TPS
TPSport = ConnectTPS(COMPort, dB);

%% Configure TPS for measurements
setPropertiesTPS(TPSport, prism_type, target_type, atr_state, hz_tol, v_tol);

%% Begin tracking
begin_time_str = datestr(now,'yyyymmddHHMMSS'); % get current time

% figure for listening the keyboard event
figure(1);
plot3(0,0,0,'o','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',12);
hold on;
grid on;
axis equal;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
title('Enter E on the keyboard to terminate the tracking', 'Fontname', 'Times New Roman','FontSize',16); % but remember not to press 'e' too many times
xlabel('X(m)','Fontname', 'Times New Roman','FontSize',14);
ylabel('Y(m)','Fontname', 'Times New Roman','FontSize',14);
zlabel('Z(m)','Fontname', 'Times New Roman','FontSize',14);
pause(1.0); % pause for 1 second

meas_polar=[];
meas_cart=[];
meas_ts=[];
pc_ts = [];
meas_status =[]; %1: ok, 0: warning, -1: error
meas_count = 0;

track_status = trackPrism(TPSport); % track the prism ?begining)

if(strcmpi(get(gcf,'CurrentCharacter'),'e'))
   disp('Please press any key except for [E]'); 
   pause(5.0);
end

% check face (if is face II, change to face I)
% changeFace(TPSport);

error_count=0; 
error_count_thre=50;  % tolerance for the continous measurements with problem

% keep take measurements (both in polar and cartesian coordinate systems)
while(1)
    
    pause(0.001); % wait for 1 ms
    
    meas_count=meas_count+1;
    fprintf('Measurement [%s]\n',num2str(meas_count));
    cur_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
    cur_pc_ts = [str2double(cur_pc_ts_str(1:4)), str2double(cur_pc_ts_str(5:6)), str2double(cur_pc_ts_str(7:8)), str2double(cur_pc_ts_str(9:10)), str2double(cur_pc_ts_str(11:12)), str2double(cur_pc_ts_str(13:14)), str2double(cur_pc_ts_str(15:17))];
     fprintf('PC commanding timestamp: %04.0f/%02.0f/%02.0f  %02.0f:%02.0f:%02.0f.%03.0f \n', cur_pc_ts(:));  % in second , resolution: 1ms
    
    [D,Hr,V,cur_meas_ts,status] = getMeasurements(TPSport, distmode);
    if (status==3 || error_count > error_count_thre) % fatal
        break; % don't record this error measurement
    elseif (status==2) % error
         error_count=error_count+1; 
    else   % warning or well
          error_count=0;  % restart the count
    end
    meas_status = [meas_status; status];
    meas_polar = [meas_polar; [D,Hr,V]]; % in m, deg, deg
    [X,Y,Z]= polar2cart(D,Hr,V); % convert to cartesian coordinate system
    fprintf('x = %.4f [m]; y = %.4f [m]; z = %.4f [m]\n',X,Y,Z); % in meter
    meas_cart = [meas_cart; [X,Y,Z]]; % in m, m, m
    
    if(meas_count > 1)
         delta_t= 0.001 * abs(cur_meas_ts(end) - meas_ts(end,end));
         fprintf('delta-t for this measurement = %.2f [second]\n',delta_t); % in second
    end
    meas_ts = [meas_ts; cur_meas_ts];
    pc_ts = [pc_ts; cur_pc_ts];
    % to stop the tracking, press pause first, press [E] and then resume
    if (meas_count > 1 && strcmpi(get(gcf,'CurrentCharacter'),'e')) 
        disp('[E] is pressed, terminate the tracking.'); 
        break;
    end
    
    if (status<2)  % warning or well
         scatter3(X,Y,Z,20,'ro','filled'); % plot in real-time
    end
    
end

%% save coordinates
mkdir ('results',  begin_time_str);
save(['results' filesep begin_time_str filesep 'meas_cart_' begin_time_str '.mat'],'meas_cart');
save(['results' filesep begin_time_str filesep 'meas_polar_' begin_time_str '.mat'],'meas_polar');
save(['results' filesep begin_time_str filesep 'meas_ts_' begin_time_str '.mat'],'meas_ts');
save(['results' filesep begin_time_str filesep 'pc_ts_' begin_time_str '.mat'],'pc_ts');
save(['results' filesep begin_time_str filesep 'meas_status_' begin_time_str '.mat'],'meas_status');
disp('Save done');

%% plot results
%begin_time_str='20201115165032'; %example string (indoor dataset1)
%begin_time_str='20201115164739'; %example string (indoor dataset2)
%begin_time_str='20201123173300';
begin_time_str='20201125172042';
load(['results' filesep begin_time_str filesep 'meas_cart_' begin_time_str '.mat'],'meas_cart');
load(['results' filesep begin_time_str filesep 'meas_status_' begin_time_str '.mat'],'meas_status');
load(['results' filesep begin_time_str filesep 'meas_ts_' begin_time_str '.mat'],'meas_ts');

meas_ts_sec = meas_ts (:,1:6);
meas_ts_sec(:,6) = meas_ts_sec(:,6) + 0.001 * meas_ts (:,7); % convert to [Year Month Day Hour Minute Second.]
[meas_gps_week, meas_gps_sow, meas_gps_dow] = local2gps(meas_ts_sec, 1);
meas_gps_sow_tracked = meas_gps_sow(meas_status<2); % select the timestamp with successful tracking

% track_position=meas_cart;
% track_position(:,1) = meas_cart(:,2);
% track_position(:,2) = meas_cart(:,1);

% tracking trajectory
figure(2);
plot3(0,0,0,'o','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',12);
hold on;
plottraj(meas_cart, meas_status);
grid on;
axis equal;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
xlabel('X(m)','Fontname', 'Times New Roman','FontSize',14);
ylabel('Y(m)','Fontname', 'Times New Roman','FontSize',14);
zlabel('Z(m)','Fontname', 'Times New Roman','FontSize',14);
legend('Total station', 'Start', 'End','Fontname', 'Times New Roman','FontSize',10);
title('Tracking result','Fontname', 'Times New Roman','FontSize',16);

% tracking status
figure(3);
plot(meas_gps_sow, meas_status, 'Linewidth', 2,'Color','r');
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
set(gca,'xticklabel',num2str((get(gca,'xtick'))'));  
xlabel('GPS seconds of the week (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('tracking status','Fontname', 'Times New Roman','FontSize',14);
ylim([0,3]);
yticks([0,1,2,3]);
title('Tracking status','Fontname', 'Times New Roman','FontSize',16);

%% TODO LISTS:
% 7. check face left or face right
% 8. set tracking tolerance
% 9. GPS settings (time system)
% 10. For each error code, directly refer to the meaning according to the
% manual
% 11. Even with error, don't directly stop, you can just add a status at
% that point and leave the coordinates empty.
