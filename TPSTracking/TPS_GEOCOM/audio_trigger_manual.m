%  Manually trigger the audio
clear; clc;
%% settings
port = "COM10";  %on windows, check COM8 and COM10
% port = '/dev/ttyUSB1';  %on Linux

bd_rate = 115200; 

%% pattern 2:  10 peaks, last in 5 seconds
% we only need this trigger

s=serialport(port,bd_rate);

disp('Trigger ON');
ms_in_day = 1/24/3600/1000;
trigger_begin_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
trigger_begin_pc_ts = [str2double(trigger_begin_pc_ts_str(1:4)), str2double(trigger_begin_pc_ts_str(5:6)), str2double(trigger_begin_pc_ts_str(7:8)), str2double(trigger_begin_pc_ts_str(9:10)), str2double(trigger_begin_pc_ts_str(11:12)), str2double(trigger_begin_pc_ts_str(13:14)), str2double(trigger_begin_pc_ts_str(15:17))];
trigger_begin_pc_datenum = datenum(trigger_begin_pc_ts(1:6))+trigger_begin_pc_ts(7)*ms_in_day;

for i=1:5
    setRTS(s,true);
    pause(0.5);
    setRTS(s,false);
    pause(0.5);
end

clear s;

save(['results' filesep  'trigger_timestamp'  filesep trigger_begin_pc_ts_str  '_datenum.mat'],'trigger_begin_pc_datenum');
save(['results' filesep  'trigger_timestamp'  filesep trigger_begin_pc_ts_str  '_ts.mat'],'trigger_begin_pc_ts');
disp('Trigger Off, save begining timestamp');

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
