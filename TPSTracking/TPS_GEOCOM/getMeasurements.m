function [D, Hz, V, ts] = getMeasurments(TPSport, distmode)
% getMeasurments : get the measurements in spherical coordinate system with the timestamp
% ts (unit: second, resolution: 0.01 s)

    % Get the measurement timestamp
        fprintf('CSV_GetDateTimeCentiSec - returning angles and distance measurement\n');
        req='%R1Q,5117:';
        fprintf (TPSport, req); 
        
        % Reply - read response data into the sting-variable out        
        out = fscanf (TPSport);
        data = regexp(out,':(?<RC>[^,]*),(?<Year>[^,]*),(?<Month>[^,]*),(?<Day>[^,]*),(?<Hour>[^,]*),(?<Minute>[^,]*),(?<Second>[^,]*),(?<CentiSecond>[^,]*)','names');
        
        ts_status =str2num(data.RC);
        
        if(ts_status==0)
            ts=[data.Year data.Month data.Day data.Hour data.Minute data.Second '.' data.CentiSecond];
            ts = str2double(ts);
        end
    
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
            Hz = Hz * 180/pi; % in deg
            V= 90 - (Z * 180/pi);    % in deg (vertical angle)

     
% Print the values
            fprintf('D = %.4f [m]; Hz = %.4f [deg]; V = %.4f [deg]\n',D, Hz, V); % in deg
            if(meas_status == 1284)
                fprintf('[WARNING] Accuracy is not guaranteed, because the result are consist of measuring data which accuracy could not be verified by the system. Co-ordinates are available.'); 
            end
            if(meas_status == 1283)
                fprintf('[WARNING]The results are not corrected by all active sensors. Co-ordinates are available.'); 
            end  
        else
            fprintf('[ERROR] GRC_Return-Code= [%s]; Check the GRC return-code in manual!\n', data.RC);
        end
    end
end
