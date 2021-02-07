%% Use this function "take_single_measurement" to do the total station resection
% By Yue Pan @ ETHZ IGP

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
% dB = 115200;
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
hz_tol = 1.57079e-05; % Horizontal tolerance (moderate resolution) 
v_tol = 1.57079e-05; % Vertical tolerance (moderate resolution)

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

distmode = 2; % Default distance mode (from BAP_SetMeasPrg)
 
%% Open port, connect to TPS
TPSport = connect_tps(COMPort, dB);

%% Configure TPS for measurements
set_properties_tps(TPSport, prism_type, target_type, atr_state, hz_tol, v_tol);

%% Take measurement (Face I)
begin_time_str = datestr(now,'yyyymmddHHMMSS'); % get current time
 [D,Hr,V,ts,status] = getMeasurements(TPSport, distmode);
 [X1,Y1,Z1]= polar2cart(D,Hr,V); % convert to cartesian coordinate 
 fprintf('Face I: x = %.4f [m]; y = %.4f [m]; z = %.4f [m]\n',X1,Y1,Z1); % in meter

%% Change face
change_face(TPSport); 

%% Take measurement (Face II)
 [D,Hr,V,ts,status] = getMeasurements(TPSport, distmode);
 [X2,Y2,Z2]= polar2cart(D,Hr,V); % convert to cartesian coordinate system
  fprintf('Face II: x = %.4f [m]; y = %.4f [m]; z = %.4f [m]\n',X2,Y2,Z2); % in meter
  
 %% Change face back
change_face(TPSport); 

 %% Take average and then record the point
meas_cart=0.5*( [X1,Y1,Z1]+ [X2,Y2,Z2]);

mkdir ('results',  'single_point');
save(['results' filesep 'single_point' filesep 'single_point_' begin_time_str '.mat'],'meas_cart');
 
 