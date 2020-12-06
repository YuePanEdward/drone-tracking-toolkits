%  Serial communication

%% settings
port = "COM8";  %on windows
% port = '/dev/ttyUSB1';  %on Linux

bd_rate = 115200; 

%% module 1

s=serialport(port,bd_rate);

begin_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
begin_pc_ts = [str2double(begin_pc_ts_str(1:4)), str2double(begin_pc_ts_str(5:6)), str2double(begin_pc_ts_str(7:8)), str2double(begin_pc_ts_str(9:10)), str2double(begin_pc_ts_str(11:12)), str2double(begin_pc_ts_str(13:14)), str2double(begin_pc_ts_str(15:17))];

setRTS(s,true);
pause(5); % pause 5 seconds

end_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
end_pc_ts = [str2double(end_pc_ts_str(1:4)), str2double(end_pc_ts_str(5:6)), str2double(end_pc_ts_str(7:8)), str2double(end_pc_ts_str(9:10)), str2double(end_pc_ts_str(11:12)), str2double(end_pc_ts_str(13:14)), str2double(end_pc_ts_str(15:17))];
setRTS(s,false);

clear s;


%% module 2

% s=serialport(port,bd_rate);
% 
% for i=1:10
%     setRTS(s,true);
%     pause(0.5);
%     setRTS(s,false);
%     pause(0.5);
% end
% 
% clear s;

%%
% writeline(s,"*IDN?")
% s.NumBytesAvailable
% 
% idn = readline(s)
% 
% write(s,"Data:Destination RefB","string");
% write(s,"Data:Encdg SRPbinary","string");
% write(s,"Data:Width 2","string");
% write(s,"Data:Start 1","string");
% 
% clear s;