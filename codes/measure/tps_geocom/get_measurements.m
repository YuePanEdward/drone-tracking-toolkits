function [D, Hz, V, ts_meas_datenum, status] = get_measurements(TPSport, distmode)
% get_measurments : get the measurements in spherical coordinate system with the timestamp
% By Yue Pan @ ETHZ IGP
% outputs:
% - D: range measurement (m)
% - Hz: horizontal angle (deg)
% - V: vertical angle (deg)
% - [Deprecated] ts_meas: timestamp (unit: second, resolution: 0.01 s, format: Local time[UTC+], [YYYY, MM, DD, MM, SS, CS] )
% - ts_meas_datenum: approximate measurement timestamp in Central PC's time system with the datenum format (unit: day)
% - status: tracking status indicator, 0: ok, 1: warning, 2: error, 3: fatal
    
    ms_in_day = 1/24/3600/1000;
    status=0; % 0: ok, 1: warning, 2: error, 3: fatal
    
    %ts_begin = TPSnow(TPSport);
    cur_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
    ts_begin = [str2double(cur_pc_ts_str(1:4)), str2double(cur_pc_ts_str(5:6)), str2double(cur_pc_ts_str(7:8)), str2double(cur_pc_ts_str(9:10)), str2double(cur_pc_ts_str(11:12)), str2double(cur_pc_ts_str(13:14)), str2double(cur_pc_ts_str(15:17))];
    ts_begin_datenum = datenum(ts_begin(1:6))+ts_begin(7)*ms_in_day;
    %fprintf('Begin command timestamp: %04.0f/%02.0f/%02.0f  %02.0f:%02.0f:%02.0f.%03.0f \n', ts_begin(:)); 
       
    % Take angle and distance measurments
    if ~isempty(distmode)   
        fprintf('BAP_MeasDistanceAngle - returning angles and distance measurement\n'); 
        req=['%R1Q,17017:', num2str(distmode)]; % 14 byte request 
        fprintf (TPSport, req); 
        
        pc_2_tps_tran_time = 14*8/TPSport.BaudRate*1e3; % unit: ms
        
        % Reply - read response data into the sting-variable out
        out = fscanf (TPSport);

        % Extract the returned values
        data = regexp(out,':(?<RC>[^,]*),(?<Hz>[^,]*),(?<V>[^,]*),(?<D>[^,]*)','names'); % 68 byte return 
         tps_2_pc_tran_time = 68*8/TPSport.BaudRate*1e3; % unit: ms 
        
        meas_status =str2double(data.RC);
        
        % Check output and store result
        if (meas_status == 0 || meas_status == 1284 || meas_status ==1283)
            Hz = str2double(data.Hz); % in rad
            Z = str2double(data.V); % in rad (zenith distance)
            D = str2double(data.D); % in m
            Hz = Hz * 180/pi;       % in deg (horziontal angle)
            V= 90 - (Z * 180/pi);   % in deg (vertical angle)
            
            % Print the values
            fprintf('D = %.4f [m]; Hz = %.4f [deg]; V = %.4f [deg]\n',D, Hz, V); % in deg
            
            % Deal with exceptions
            if(meas_status == 1284)
                fprintf('[WARNING] Accuracy is not guaranteed, because the result are consist of measuring data which accuracy could not be verified by the system. Coordinates are available.\n'); 
                status=1; % warning            
            end
            if(meas_status == 1283)
                fprintf('[WARNING] The results are not corrected by all active sensors. Coordinates are available.\n'); 
                status=1; % warning  
            end  
        else
            fprintf('[ERROR] GRC_Return-Code= [%s]; Check the meaning of the GRC return-code in manual [page 81]!\n', data.RC);
            D=0; Hz=0; V=0; 
            status=2; % error  
        end
    end
   
    fprintf('Tracking status [%d], GRC_Returen-Code= [%s]\n', status, data.RC);
    
   %ts_end = tps_now(TPSport);
    
   cur_pc_ts_str = datestr(now,'yyyymmddHHMMSSFFF'); % get timestamp on PC
   ts_end = [str2double(cur_pc_ts_str(1:4)), str2double(cur_pc_ts_str(5:6)), str2double(cur_pc_ts_str(7:8)), str2double(cur_pc_ts_str(9:10)), str2double(cur_pc_ts_str(11:12)), str2double(cur_pc_ts_str(13:14)), str2double(cur_pc_ts_str(15:17))];
   ts_end_datenum = datenum(ts_end(1:6))+ts_end(7)*ms_in_day;
   %fprintf('End command timestamp: %04.0f/%02.0f/%02.0f  %02.0f:%02.0f:%02.0f.%03.0f \n', ts_end(:)); 
   
   ts_diff_ms = (ts_end_datenum-ts_begin_datenum)/ms_in_day; % unit: ms
   fprintf('duration for this measurement = %.0f [ms]\n',ts_diff_ms); 
   
   ts_meas_datenum = 0.5 * (ts_begin_datenum+ts_end_datenum+(pc_2_tps_tran_time-tps_2_pc_tran_time)*ms_in_day);
   ts_meas =  datevec(ts_meas_datenum); 
   ts_meas = [ts_meas, (ts_meas(6)-floor(ts_meas(6)))*1e3];
   ts_meas(6) = floor(ts_meas(6));
   fprintf('Approximate measurement timestamp: %04.0f/%02.0f/%02.0f  %02.0f:%02.0f:%02.0f.%03.0f \n', ts_meas(:)); 
    
end
