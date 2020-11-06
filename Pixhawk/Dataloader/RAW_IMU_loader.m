clear;

%log_filename='test_log2.txt';
log_filename='2019-11-02 16-14-04.txt';

log_file_path=['.' filesep 'test_data' filesep log_filename];

fid=fopen(log_file_path);
raw_data = textscan(fid,'%s'); % solve the problem for some log files which are segmented with an extra space or tab
raw_data = raw_data{1,1};
fclose(fid); 

imu_measure_count=0;

time_boot_ms=[];
time_usec=[];
xacc=[];
yacc=[];
zacc=[];
xgyro=[];
ygyro=[];
zgyro=[];
xmag=[];
ymag=[];
zmag=[];

for i=1:size(raw_data,1)
   current_str = raw_data{i};
   len_str = size(current_str,2);
   if (len_str < 7)
       continue; 
   end
   current_str_head = current_str(1:7);
   
   % Read raw IMU data
   % time_boot_ms time_usec	xacc yacc zacc xgyro ygyro zgyro xmag ymag zmag
   if(current_str_head == 'RAW_IMU')
       cur_str_split = split(current_str,',');
       cur_str_split = str2double(cur_str_split);
       time_boot_ms = [time_boot_ms; cur_str_split(2)];
       time_usec = [time_usec; cur_str_split(3)];
       xacc=[xacc; cur_str_split(4)];
       yacc=[yacc; cur_str_split(5)];
       zacc=[zacc; cur_str_split(6)];
       xgyro=[xgyro; cur_str_split(7)];
       ygyro=[ygyro; cur_str_split(8)];
       zgyro=[zgyro; cur_str_split(9)];
       xmag=[xmag; cur_str_split(10)];
       ymag=[ymag; cur_str_split(11)];
       zmag=[zmag; cur_str_split(12)];
       
       imu_measure_count=imu_measure_count+1;
   end
end

disp(['Collect [', num2str(imu_measure_count), '] IMU data.']); 

figure(1);
plot(time_usec, [xacc yacc zacc]);
legend('ax','ay','az');
xlabel('timestamp (us)');
ylabel('acceleration (0.01 m/s^2)');
title('IMU accelerometer measurements');

figure(2);
plot(time_usec, [xgyro ygyro zgyro]);
legend('wx','wy','wz');
xlabel('timestamp (us)');
ylabel('angular velocity (deg/s)'); % figure out the correct unit
title('IMU gyroscope measurements');

figure(3);
plot(time_usec, [xmag ymag zmag]);
legend('mx','my','mz');
xlabel('timestamp (us)');
ylabel('don t know'); % figure out the correct unit
title('IMU magnetometer measurements');
