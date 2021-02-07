function track_status = track_prism(TPSport)
% track_prism: use GEOCOM to track the prsim
% By Yue Pan @ ETHZ IGP 
% 

% Example
% % AUS_GetUserLockState
% % Require
% req = '%R1Q,18008:';
% fprintf (TPSport, req);
% % Reply - read response data into the sting-variable out
% out = fscanf(TPSport);
% % Response
% res = sprintf('%R1P,0,0:RC,%s',num2str(0));
% % Extract the returned values
% data = regexp(out,res,'names');

lock_on_off=1;

% AUS_SetUserLockState
% Require
req=['%R1Q,18007:',num2str(lock_on_off)];
fprintf(TPSport, req);
% Reply - read response data into the sting-variable out
out = fscanf(TPSport);
% Extract the returned values
data = regexp(out,':(?<RC>[^,]*)','names');
%data = regexp(out,'%R1P,0,0:RC','names');

if(str2num(data.RC) == 0)   
    fprintf ('Tracking module available\n');
    
    % AUT_LockIn
    % Require
    req = '%R1Q,9013:';
    fprintf (TPSport, req);
    % Reply - read response data into the sting-variable out
    out = fscanf(TPSport);
    % Extract the returned values
    data = regexp(out,':(?<RC>[^,]*)','names');
    %data = regexp(out,'%R1P,0,0:RC','names');
     
    if(str2num(data.RC) == 0)   
        fprintf ('Tracking module executes successful\n');
    else
        fprintf ('[ERROR] [%.0f] something wrong with the tracking\n',data.RC);
    end
else
    fprintf ('[ERROR] Tracking module not available\n');
end

track_status=data.RC;

end

