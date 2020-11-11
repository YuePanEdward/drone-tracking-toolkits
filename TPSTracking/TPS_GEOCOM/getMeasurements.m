function [D, Hz, V, ts, status] = getMeasurments(TPSport, distmode)
% getMeasurments : get the measurements in spherical coordinate system with the timestamp
% outputs:
% - D: range measurement (m)
% - Hz: horizontal angle (deg)
% - V: vertical angle (deg)
% - ts: (unit: second, resolution: 0.01 s, format: HHMMSS.CS)
% - status: status indicator, 1: ok, 0: warning, -1: error
    
    status=1; % 1: ok, 0: warning, -1: error
    % Get the measurement timestamp
    fprintf('CSV_GetDateTimeCentiSec - returning timestamp of the TPS\n');
    req='%R1Q,5117:';
    fprintf (TPSport, req); 
        
    % Reply - read response data into the sting-variable out        
    out = fscanf (TPSport);
    data = regexp(out,':(?<RC>[^,]*),(?<Year>[^,]*),(?<Month>[^,]*),(?<Day>[^,]*),(?<Hour>[^,]*),(?<Minute>[^,]*),(?<Second>[^,]*),(?<CentiSecond>[^,]*)','names');
    ts_status =str2num(data.RC);
        
    if(ts_status==0)
        %ts=[data.Year data.Month data.Day data.Hour data.Minute data.Second '.' data.CentiSecond];
        ts = [data.Hour data.Minute data.Second '.' data.CentiSecond];
        ts = str2double(ts);
    end
    fprintf('timestamp = %.2f [second]\n', ts); % in second , resolution: 10ms
    
    % Take angle and distance measurments
    
    if ~isempty(distmode)   
        fprintf('TMC_GetAngle - returning angles and distance measurement\n');
        req=['%R1Q,17017:', num2str(distmode)];
        fprintf (TPSport, req); 

        % Reply - read response data into the sting-variable out
        out = fscanf (TPSport);

        % Extract the returned values
        data = regexp(out,':(?<RC>[^,]*),(?<Hz>[^,]*),(?<V>[^,]*),(?<D>[^,]*)','names');
        
        meas_status =str2num(data.RC);
        % Check output and store result
        if (meas_status == 0 || meas_status == 1284 || meas_status ==1283)
            Hz = str2double(data.Hz); % in rad
            Z = str2double(data.V); % in rad (zenith distance)
            D = str2double(data.D); % in m
            Hz = Hz * 180/pi;       % in deg (horziontal angle)
            V= 90 - (Z * 180/pi);   % in deg (vertical angle)
            
            % Print the values
            fprintf('D = %.4f [m]; Hz = %.4f [deg]; V = %.4f [deg]\n',D, Hz, V); % in deg
            if(meas_status == 1284)
                fprintf('[WARNING] Accuracy is not guaranteed, because the result are consist of measuring data which accuracy could not be verified by the system. Co-ordinates are available.'); 
                status=0; % warning            
            end
            if(meas_status == 1283)
                fprintf('[WARNING] The results are not corrected by all active sensors. Co-ordinates are available.'); 
                status=0; % warning  
            end  
        else
            fprintf('[ERROR] GRC_Return-Code= [%s]; Check the GRC return-code in manual!\n', data.RC);
            D=0; Hz=0; V=0; 
            status=-1; % error  
        end
    end
end
