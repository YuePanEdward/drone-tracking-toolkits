function [D, Hz, V] = getMeasurments(TPSport, distmode)
% getMeasurments: get the measurement in spherical coordinate system with the timestamp

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