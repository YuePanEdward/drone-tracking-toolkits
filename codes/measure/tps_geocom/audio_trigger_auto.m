% Automaticlly trigger the audio or flash signal
% For the radio-synchronized network of audio/flash triggers 
% By Yue Pan @ ETHZ IGP

clear; clc;
%% settings
% COM port number (check in the device manager)
port = "COM6";  %on windows, check COM8 and COM10
% port = '/dev/ttyUSB1';  %on Linux
% On Linux, you need to firstly give the access to the ttyUSBx port by
%  sudo chmod 666 /dev/ttyUSB0

bd_rate = 115200; 

% always consider to use the audio_trigger_manual to manually trigger the
% audio

%% pattern:  10 peaks, lasts in 5 seconds.  repeat it for every 30 seconds

s=serialport(port,bd_rate);


% figure for listening the keyboard event
figure(99);
title('Enter E on the keyboard to terminate the triggering', 'Fontname', 'Times New Roman','FontSize',16); % but remember not to press 'e' too many times
pause(0.1); % pause for 0.1 second

count=0;
task_begin_pc_ts_str='000';

% keep doing so % duration: 30 seconds 
while(1)
get(gcf,'CurrentCharacter');
 % to stop the triggering, press pause first, press [E] and then resume
    if (count > 0 && strcmpi(get(gcf,'CurrentCharacter'),'e')) 
          disp('[E] is pressed, terminate the automatic triggering.'); 
          break;
    end
        
count =count +1;
disp(['[', num2str(count), '], Trigger ON']);

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

% create the task folder (sub-folder version)
if (count == 1)
    task_begin_pc_ts_str = trigger_begin_pc_ts_str;
    mkdir (['results' filesep 'trigger_timestamp'] , task_begin_pc_ts_str); 
end


% sub-folder version
save(['results' filesep  'trigger_timestamp'  filesep task_begin_pc_ts_str filesep trigger_begin_pc_ts_str  '_datenum.mat'],'trigger_begin_pc_datenum');
save(['results' filesep  'trigger_timestamp'  filesep task_begin_pc_ts_str filesep trigger_begin_pc_ts_str  '_ts.mat'],'trigger_begin_pc_ts');

disp(['[', num2str(count)  , '], Trigger Off, save begining timestamp']);

pause(25.0);  % wait for 25 seconds

end

clear s;