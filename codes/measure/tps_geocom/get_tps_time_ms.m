function [cur_tps_ts] = get_tps_time_ms(TPSport, distmode)
% get_tps_time_ms: get TPS internal timestamp with ms level resolution
% By Yue Pan @ ETHZ IGP

if ~isempty(distmode)   
    % Try this to get more accurate timestamp using GetAngle1
    fprintf('TMC_GetAngle1 - returning comprehensive angles measurement\n');
    req=['%R1Q,2003:', num2str(distmode)];
    fprintf (TPSport, req); 
         
    % Reply - read response data into the sting-variable out
    out = fscanf (TPSport);

    % Extract the returned values
    %data = regexp(out,':(?<RC>[^,]*),(?<Hz>[^,]*),(?<V>[^,]*),(?<AngleAccuracy>[^,]*),(?<AngleTime>[^,]*),(?<CrossIncline>[^,]*),(?<LengthIncline>[^,]*),(?<AccuracyIncline>[^,]*),(?<InclineTime>[^,]*),(?<FaceDef>[^,]*)','names');
    data = regexp(out,':(?<RC>[^,]*),(?<Hz>[^,]*),(?<V>[^,]*),(?<AngleAccuracy>[^,]*),(?<AngleTime>[^,]*)','names'); 
    
    
    meas_status =str2double(data.RC);
         
    fprintf('measurement status: [%d]\n', meas_status);
    
    cur_tps_ts = 0;
    
    if (meas_status == 0)
         cur_tps_ts = str2double(data.AngleTime)
    end
    % Check output and store result
     if (meas_status == 0 || meas_status == 1284 || meas_status ==1283)
             Hz = str2double(data.Hz); % in rad
             Z = str2double(data.V); % in rad (zenith distance)
             accuracy = str2double(data.AngleAccuracy);
             angle_ts = str2double(data.AngleTime);
             Hz = Hz * 180/pi;       % in deg (horziontal angle)
             V= 90 - (Z * 180/pi);   % in deg (vertical angle)   
             
             fprintf('Angle measurement timestamp [%s]\n', data.AngleTime);
         end
     end
end

end

