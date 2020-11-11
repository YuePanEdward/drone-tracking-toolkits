% Matlab codes for the automatic control of a Leica total station (TS50/60,
% TPS 1200, etc.)
% By YUE PAN @ ETHZ IGP 
% IPA project: Measuring Drone Trajectory using Total Stations with Visual Tracking

% TODO: add more comments, refine the code

%%
clear; clc; close all;
addpath(['..' filesep '..' filesep 'common']);

%% Set GeoCOM port, dB (Baud) rate
% For TPS1200, set it in configure->interfaces setting->GSI/GeoCOM mode on
% For Nova TS 50/60, set it in (TODO)

% COM port number (check in the device manager)
% COMPort = '/dev/ttyUSB1';  %on Linux
COMPort = 'COM4';            %on Windows

% dB (Baud) rate
dB = 115200;
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
prism_type = 3; 

%% Set TARGET TYPE
% REFLECTOR_TARGET = 0
% REFLECTORLESS_TARGET = 1
target_type = 0; 

%%
atr_state = 1; % ATR state on (1)

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

distmode = 4; % Default distance mode (from BAP_SetMeasPrg)
 
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
title('Enter E on the keyboard to terminate the tracking'); % but remember not to press 'e' too many times
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
pause(2.0); % pause for 1 second

meas_polar=[];
meas_cart=[];
meas_ts=[];
meas_status =[]; %1: ok, 0: warning, -1: error
meas_count = 0;

track_status = trackPrism(TPSport); % track the prism ?begining)

if(strcmpi(get(gcf,'CurrentCharacter'),'e'))
   disp('Please press any key except for [E]'); 
   pause(3.0);
end

% check face (if is face II, change to face I)
% changeFace(TPSport);

% keep take measurements (both in polar and cartesian coordinate systems)
while(1)
    
    pause(0.001); % wait for 1 ms
    
    meas_count=meas_count+1;
    fprintf('Measurement [%s]\n',num2str(meas_count));
    %meas_ts =[meas_ts; str2num(datestr(now,'HHMMSSFFF'))]; % get approximate timestamp
    [D,Hr,V,ts,status] = getMeasurements(TPSport, distmode);
    if (status<0) % error
        break; % don't record this error measurement
    end
    meas_status = [meas_status,status];
    meas_polar = [meas_polar;[D,Hr,V]]; % in m, deg, deg
    [X,Y,Z]= polar2cart(D,Hr,V); % convert to cartesian coordinate system
    fprintf('x = %.4f [m]; y = %.4f [m]; z = %.4f [m]\n',X,Y,Z); % in meter
    meas_cart = [meas_cart; [X,Y,Z]]; % in m, m, m
    if(meas_count > 1)
      delta_t= abs(ts- meas_ts(end));
      fprintf('delta-t for this measurement = %.2f [second]\n',delta_t); % in meter
    end
    meas_ts = [meas_ts; ts];
    
    % to stop the tracking, press pause first, press [E] and then resume
    if (meas_count > 1 && strcmpi(get(gcf,'CurrentCharacter'),'e')) 
        disp('[E] is pressed, terminate the tracking.'); 
        break;
    end
 
    scatter3(X,Y,Z,20,'ro','filled'); % plot in real-time
end

%% save coordinates
save(['results' filesep 'meas_cart_' begin_time_str '.mat'],'meas_cart');
save(['results' filesep 'meas_polar_' begin_time_str '.mat'],'meas_polar');
save(['results' filesep 'meas_ts_' begin_time_str '.mat'],'meas_ts');
disp('Save done');

%% plot results
%begin_time_str='20201106171455'; %example string (indoor dataset1)
%begin_time_str='20201106183639'; %example string (indoor dataset2)
load(['results' filesep 'meas_cart_' begin_time_str '.mat'],'meas_cart');

figure(2);
plot3(0,0,0,'o','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',12);
hold on;
plottraj(meas_cart);
grid on;
axis equal;
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
legend('Total station', 'Beginning', 'Ending');
title('Tracking result');

% figure(2);
% plot3(meas_cart(2:end-1,1),meas_cart(2:end-1,2),meas_cart(2:end-1,3),'r:o','MarkerSize',4); % plot the final trajectory
% hold on;
% plot3(meas_cart(1,1),meas_cart(1,2),meas_cart(1,3),'go','MarkerFaceColor','g','MarkerSize',10); % plot the begining point
% plot3(meas_cart(end,1),meas_cart(end,2),meas_cart(end,3),'bo','MarkerFaceColor','b','MarkerSize',10); % plot the ending point
% plot3(0,0,0,'o','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',12);
% grid on;
% xlabel('X(m)');
% ylabel('Y(m)');
% zlabel('Z(m)');
% legend('Prism position', 'Beginning', 'Ending', 'Total station');
% title('Tracking result');

%% TODO LISTS:
% 7. check face left or face right
% 8. set tracking tolerance
% 9. GPS settings
