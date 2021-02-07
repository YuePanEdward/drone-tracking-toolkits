function [ts] = tps_now(TPSport)
% tps_now: get current TPS timestamp in TPS system (resolution: 10ms)
% By Yue Pan @ ETHZ IGP
    
    ts = -1 *ones(1,7);
     
    %fprintf('CSV_GetDateTimeCentiSec - returning timestamp of the TPS\n');
    req='%R1Q,5117:'; % byte number: 12
    fprintf (TPSport, req); 
     
    % Reply - read response data into the sting-variable out        
    out = fscanf (TPSport);
    % '%R1P,0,0: 8*short
    % byte number: 34
    
    
    data = regexp(out,':(?<RC>[^,]*),(?<Year>[^,]*),(?<Month>[^,]*),(?<Day>[^,]*),(?<Hour>[^,]*),(?<Minute>[^,]*),(?<Second>[^,]*),(?<CentiSecond>[^,]*)','names');
    ts_status =str2double(data.RC);
        
    if(ts_status==0)
        % [year month day hour min sec msec] 1*7 mat
        ts=[str2double(data.Year),  str2double(data.Month),  str2double(data.Day),  str2double(data.Hour),  str2double(data.Minute), str2double(data.Second) , 10*str2double(data.CentiSecond)]; % convert to ms 
        fprintf('TPS timestamp: %04.0f/%02.0f/%02.0f  %02.0f:%02.0f:%02.0f.%03.0f \n', ts(:));  % in second , resolution: 1ms
    end
    
    %transmit_time_delay = 46 * 8 / TPSport.BaudRate * 1e3; %unit (ms)  
    
end

