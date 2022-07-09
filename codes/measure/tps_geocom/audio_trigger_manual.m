%  Manually trigger the audio or flash signal (for test before the flight)

% For the radio-synchronized network of audio/flash triggers 
% By Yue Pan @ ETHZ IGP
clear; clc;
%% settings
% COM port number (check in the device manager)
% port = "COM6";  %on windows, check COM8 and COM10
% On Linux, you need to firstly give the access to the ttyUSBx port by
%  sudo chmod 666 /dev/ttyUSB0
%  sudo chmod 666 /dev/ttyUSB1
% ls /dev/ to check which ports are available

% Remember to use the Uart Trigger of the new framework (the one with five
% buttons)

% And try to firstly plug in the Xbee module and then the box trigger
% because the camera_operator_api.py use ttyUSB0 as its defualt port (that
% should belongs to Xbee module)

% Maybe use ttyUSB2 for the Total station

port = '/dev/ttyUSB2';  %on Linux

bd_rate = 115200; 

%% pattern 1:  10 peaks, last in 10 seconds (on - off -on -off ...)
% we only need this trigger

duration = 5; % unit would be second here
%duration = 20; % unit would be second here
% duration = 200;

s=serialport(port,bd_rate);

disp('Trigger ON');
ms_in_day = 1/24/3600/1000;
% only record the begining timestamp
trigger_begin_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
trigger_begin_pc_ts = [str2double(trigger_begin_pc_ts_str(1:4)), str2double(trigger_begin_pc_ts_str(5:6)), str2double(trigger_begin_pc_ts_str(7:8)), str2double(trigger_begin_pc_ts_str(9:10)), str2double(trigger_begin_pc_ts_str(11:12)), str2double(trigger_begin_pc_ts_str(13:14)), str2double(trigger_begin_pc_ts_str(15:17))];
trigger_begin_pc_datenum = datenum(trigger_begin_pc_ts(1:6))+trigger_begin_pc_ts(7)*ms_in_day;

% always on
% setRTS(s,false);
% pause(1000);

% % flashing (1Hz, 10 seconds, 0.5s On, 0.5s Off)
 for i=1:duration
    setRTS(s,true);
    pause(0.5);
    setRTS(s,false);
    pause(0.5);
 end

setRTS(s,false);

% 
% % set to false in the end
%  for i=1:duration
% setRTS(s,false);
% pause(0.5);
%  end
% pause(1000);


% flashing (1Hz, 20 seconds, 0.1s On, 0.9s Off)
% for i=1:50
%     setRTS(s,true);
%     pause(0.2);
%     setRTS(s,false);
%     pause(0.8);
% end

clear s;
disp('Trigger Off, save begining timestamp');
save(['results' filesep  'trigger_timestamp'  filesep trigger_begin_pc_ts_str  '_datenum.mat'],'trigger_begin_pc_datenum');
save(['results' filesep  'trigger_timestamp'  filesep trigger_begin_pc_ts_str  '_ts.mat'],'trigger_begin_pc_ts');


%% pattern 1
% for test only
% disp('Trigger ON');
% s=serialport(port,bd_rate);
% 
% begin_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
% begin_pc_ts = [str2double(begin_pc_ts_str(1:4)), str2double(begin_pc_ts_str(5:6)), str2double(begin_pc_ts_str(7:8)), str2double(begin_pc_ts_str(9:10)), str2double(begin_pc_ts_str(11:12)), str2double(begin_pc_ts_str(13:14)), str2double(begin_pc_ts_str(15:17))];
% 
% setRTS(s,true);
% pause(5); % pause 5 seconds
% 
% clear s;
