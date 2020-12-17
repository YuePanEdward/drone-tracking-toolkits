%%
clear; clc; close all;
addpath(['..' filesep '..' filesep 'common']);

%% sync time£º Central PC time
% sequence for data processing
% drone_data_loader --> get_tran_wt --> tran_track

%%  Load the drone's pose estimated by onboard GNSSINS
%log_filename='2020-11-23 17-30-31';  %test_7
log_filename='2020-11-25 17-21-24';  %test_8

second_in_day = 24 * 3600;
project_origin_datenum = 7.381207227069444e+05; % load it

%%
load(['results' filesep log_filename filesep log_filename '_position_ekf.mat'],'track_position_ekf'); % E, N, U
load(['results' filesep log_filename filesep log_filename '_attitude_ekf.mat'],'track_attitude_ekf'); % roll, pitch, yaw
load(['results' filesep log_filename filesep log_filename '_localtime_ekf.mat'],'track_local_ts_datenum_ekf');  % timestamp

% convert the euler angle to quaternion for interpolation
attitude_count = size(track_attitude_ekf,1);
quat_ekf = quaternion(zeros(attitude_count,4)); 
for i=1:attitude_count
    quat_ekf(i,1) = quaternion(track_attitude_ekf(i,:),'eulerd','XYZ','frame'); % sequence : x-y'-z'' % unit: degree
end

% extrinsic rotation
disp('Load the pose of the drone estimated by onboard GNSSINS system successfully');

%% Load track results
%track_transaction_id='20201123173300';   %test_7
track_transaction_id='20201125172042';   %test_8
track_result_path_base = ['..' filesep '..' filesep 'TPSTracking'  filesep 'TPS_GEOCOM' filesep 'results'];
load([track_result_path_base filesep track_transaction_id filesep 'meas_polar_' track_transaction_id '.mat'],'meas_polar');
load([track_result_path_base filesep track_transaction_id filesep 'meas_cart_' track_transaction_id '.mat'],'meas_cart');
load([track_result_path_base filesep track_transaction_id filesep 'meas_status_' track_transaction_id '.mat'],'meas_status');
load([track_result_path_base filesep track_transaction_id filesep 'meas_ts_' track_transaction_id '.mat'],'meas_ts');

% pc_tps_shift = 0.1; % unit: s
% 
% tps_ts = pc_ts (:,1:6);
% tps_ts(:,6) = tps_ts(:,6) + 0.001 * pc_ts (:,7) + pc_tps_shift;  % convert to [Year Month Day Hour Minute Second.]
% [meas_gps_week, meas_gps_sow, meas_gps_dow] = local2gps(tps_ts, 1); % 1 means UTC+1

% use central PC time for sync
tps_ts = meas_ts;

% Get the tracking segements of time of interest (toi)
%index_toi_tps_track = find(meas_gps_sow>track_time_gps_sow_ekf(1) &  meas_gps_sow<track_time_gps_sow_ekf(end) & meas_status<2);

index_toi_tps_track = find(tps_ts >track_local_ts_datenum_ekf(1) &  tps_ts< track_local_ts_datenum_ekf(end)& meas_status<2);
meas_cart_toi = meas_cart(index_toi_tps_track, :);
meas_polar_toi = meas_polar(index_toi_tps_track, :);
meas_status_toi = meas_status(index_toi_tps_track, :);
tps_ts_toi = tps_ts(index_toi_tps_track, :);
toi_sample_count = size(meas_cart_toi,1);

disp('Load the total station tracking position of the prism successfully');

%% Load the transformation matrix from total station to local reference coordinate system
load(['results' filesep log_filename filesep log_filename '_tran_mat_tps2local.mat'],'tran_mat_tps2local');
load(['results' filesep log_filename filesep log_filename '_cov_mat_tps2local.mat'],'cov_mat_tps2local');
% cov_mat_tps2local {sequence: (tx,ty,tz,roll,pitch,yaw) }
disp('Load georeference transformation successfully');

%% Apply the transformation

tran_mat = [tran_mat_tps2local; 0 0 0 1];
track_points_tps = [meas_cart_toi' ; ones(1,toi_sample_count)]; % [4 * n]
track_points_local = tran_mat * track_points_tps; % [4 * n]
track_points_local = track_points_local(1:3,:)';  % [n * 3]

disp('Apply the georeference transformation successfully');

%% Temprol interprelation of attitude based on uniform motion model

track_points_euler_rad = zeros(toi_sample_count, 3);

for i=1:toi_sample_count
    cur_tps_ts = tps_ts_toi(i);
    % find the adjacent quaternions of each tracked position according to the local timestamp for interpolation 
    % use slerp
    delta_t = track_local_ts_datenum_ekf - cur_tps_ts;
    indices_pre = find(delta_t<=0);
    index_pre = indices_pre(end);
    indices_post = find(delta_t>0);
    index_post = indices_post(1);
    interpolate_ratio = (cur_tps_ts - track_local_ts_datenum_ekf(index_pre)) / (track_local_ts_datenum_ekf(index_post) - track_local_ts_datenum_ekf(index_pre)); 
    interpolate_quat = slerp(quat_ekf(index_pre),quat_ekf(index_post),interpolate_ratio); 
    interpolate_euler = quat2eul(interpolate_quat, 'XYZ'); % unit:rad
    track_points_euler_rad(i,:) = interpolate_euler; % unit:rad
end
disp('Conduct the sychronization successfully');

%% Get the accurate position of the drone 
% use the leverarm
leverarm = [-0.102; 0; 0.19];
tran_enu2ned = [0 1 0; 1 0 0; 0 0 -1];
z_coordinate_system_gap = 6.5; % important, due to the conversion between LV95 and WGS84

track_drone_centre_local = zeros(toi_sample_count,3);
for i=1:toi_sample_count
     R_lb = (eul2rotm(track_points_euler_rad(i,:),'XYZ'))'; % unit:rad
     track_drone_centre_local(i,:) =  (track_points_local(i,:)' -  tran_enu2ned * R_lb * leverarm)' ; 
     track_drone_centre_local(i,3)= track_drone_centre_local(i,3)+z_coordinate_system_gap;
end
track_points_euler_deg = 180/pi *track_points_euler_rad;

disp('Get the position of the drone centre');

%% Calculate the covariance matrix
track_points_euler_deg = 180.0 / pi *track_points_euler_rad;
cov_leverarm = diag([5e-4,5e-4,5e-4]).^2; %accuracy:0.5mm 

orientation_error_ratio_bias = 1e-3; % Take care of this parameter controling the accuracy of IMU and EKF
orientation_error_ratio_rate = 4e-4; % per second

% define Total station accuracy
 std_ang = 0.5 / 3600;  % 0.5 ''
 %std_ang = 30.0 / 3600; % 30 ''
 std_dist_m = 0.6*1e-3; % 0.6 mm
 std_dist_ppm = 1; % 1ppm

% Reference: A tutorial on SE(3) transformation parameterizations and
% on-manifold optimization [Page17, 22-23]

cov_drone_centre_list = cell(toi_sample_count, 1);
cov_prism_tps_list = cell(toi_sample_count, 1);
cov_prism_local_list = cell(toi_sample_count, 1);
cov_prism_drone_list = cell(toi_sample_count, 1);

for i=1:toi_sample_count
     R_lb = (eul2rotm(track_points_euler_rad(i,:),'XYZ'))'; % unit:rad
     cov_pose_drone =zeros(6);
     orientation_error_ratio_cur = orientation_error_ratio_bias + orientation_error_ratio_rate * (tps_ts_toi(i)-tps_ts_toi(1))*second_in_day; % edit it later
%      cov_pose_drone(4,4)=orientation_error_ratio_cur*track_points_euler_rad(i,3); % yaw
%      cov_pose_drone(5,5)=orientation_error_ratio_cur*track_points_euler_rad(i,2); % pitch
%      cov_pose_drone(6,6)=orientation_error_ratio_cur*track_points_euler_rad(i,1); % roll
     cov_pose_drone(4,4) = orientation_error_ratio_cur;
     cov_pose_drone(5,5) = orientation_error_ratio_cur;
     cov_pose_drone(6,6) = orientation_error_ratio_cur;
     cov_pose_drone = cov_pose_drone.^2;
     J_mat_prism_drone = getJacobiRp(leverarm, track_points_euler_rad(i,:));
     cov_prism_drone =  J_mat_prism_drone * cov_pose_drone * J_mat_prism_drone' + R_lb * cov_leverarm * R_lb';
     cov_prism_drone_list{i} = cov_prism_drone;
     
     
     cov_prism_tps = calPointCov(meas_polar_toi(i,1), meas_polar_toi(i,2), meas_polar_toi(i,3), std_ang, std_dist_m, std_dist_ppm);
     cov_prism_tps_list{i}=cov_prism_tps;
     
     R_lt = tran_mat_tps2local(1:3,1:3);
     R_lt_euler_deg = rotm2eul(R_lt, 'XYZ'); % unit: rad
     J_mat_prism_local = getJacobiRp(meas_cart_toi(i,:), R_lt_euler_deg);
     cov_prism_local =  J_mat_prism_local * cov_mat_tps2local * J_mat_prism_local' + R_lt * cov_prism_tps * R_lt'; 
     cov_prism_local_list{i} = cov_prism_local;
     
     cov_drone_local = cov_prism_drone+cov_prism_local; %[3 x 3] mat % since they are independent
     cov_drone_centre_list{i} = cov_drone_local;
end

%% Plot estimated error

error_total=zeros(toi_sample_count,1);
error_v=error_total;
error_h=error_total;

for i=1:toi_sample_count
     temp_error_vector = sqrt(diag(cov_drone_centre_list{i}));
     error_total(i) = norm(temp_error_vector);
     error_v(i) = temp_error_vector(3);
     error_h(i) = norm(temp_error_vector(1:2));
    
end

figure(2)
plot((tps_ts_toi-project_origin_datenum)*second_in_day, [error_total, error_v, error_h]*100, 'Linewidth', 4);
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',28);
xlabel('Timestamp (s)','Fontname', 'Times New Roman','FontSize',32);
ylabel('Error (cm)','Fontname', 'Times New Roman','FontSize',32);
xlim([40,150]);
ylim([0, 2.0]);
legend('Total', 'Vertical', 'Horizontal','Fontname', 'Times New Roman','FontSize',32);

%%
error_total=zeros(toi_sample_count,1); % drone centre in local frame
error_prism_tps= error_total;  % prism in total station frame
error_prism_local = error_total; % prism in local frame
error_leverarm = error_total;  % prism in body frame
error_prism_drone = error_total; % prism in rotated body frame
error_drone_tps = error_total;  % drone centre in total station frame

for i=1:toi_sample_count
     error_total(i) = norm(sqrt(diag(cov_drone_centre_list{i})));
     error_prism_tps(i) = norm(sqrt(diag(cov_prism_tps_list{i})));
     error_prism_local(i) =  norm(sqrt(diag(cov_prism_local_list{i})));
     error_leverarm(i) =  norm(sqrt(diag(cov_leverarm)));
     error_prism_drone(i) = norm(sqrt(diag(cov_prism_drone_list{i})));    
     error_drone_tps (i) = norm(sqrt(diag(cov_prism_tps_list{i}+cov_prism_drone_list{i})));  
end

figure(3)
plot((tps_ts_toi-project_origin_datenum)*second_in_day, [error_total, error_prism_tps, error_prism_local, error_leverarm, error_prism_drone, error_drone_tps]*100, 'Linewidth', 4);
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',28);
xlabel('Timestamp (s)','Fontname', 'Times New Roman','FontSize',32);
ylabel('Error (cm)','Fontname', 'Times New Roman','FontSize',32);
xlim([40,150]);
ylim([0, 2.0]);
legend('Total (drone center in local geo-referenced frame)', 'Prism in total station frame', 'Prism in local geo-referenced frame','Prism in drone body frame', 'Prism in rotated drone body frame','Drone center in total station frame','FontSize',32);

%% Plot  trajectory
disp('Begin Plotting');
figure(1);
tps_axis_length = 0.6;
tps_linewidth = 3.5;
% plot total station position
plot3(tran_mat_tps2local(1,4),tran_mat_tps2local(2,4),tran_mat_tps2local(3,4)+z_coordinate_system_gap,'o','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',12);
hold on;
% plot the pose of the prism (input unit:deg)
% plottraj(track_points_local, meas_status_toi, track_points_euler_deg); 
% plot the pose of the drone's centre (input unit:deg)
plottraj(track_drone_centre_local,meas_status_toi, track_points_euler_deg, cov_drone_centre_list); 
% plot the pose of the drone's centre estimated the drone's ekf
% status_vec= zeros(size(track_position_ekf,1),1);
% plottraj(track_position_ekf, status_vec, track_attitude_ekf); 
% plot total station axis
tpso = tran_mat_tps2local(:,4);
tpso(3)=tpso(3)+z_coordinate_system_gap;
po_axis = tran_mat_tps2local(:,1:3) * (tps_axis_length.* eye(3)) + repmat(tpso,[1,3]);
tpsax = po_axis(:,1); tpsay = po_axis(:,2); tpsaz = po_axis(:,3);
line([tpso(1); tpsax(1)], [tpso(2); tpsax(2)], [tpso(3); tpsax(3)], 'Color', 'r', 'LineWidth', tps_linewidth); 
line([tpso(1); tpsay(1)], [tpso(2); tpsay(2)], [tpso(3); tpsay(3)], 'Color', 'g', 'LineWidth', tps_linewidth); 
line([tpso(1); tpsaz(1)], [tpso(2); tpsaz(2)], [tpso(3); tpsaz(3)], 'Color', 'b', 'LineWidth', tps_linewidth); 
grid on;
axis equal;
set(gca, 'Fontname', 'Times New Roman','FontSize',16);
xlabel('X(m)','Fontname', 'Times New Roman','FontSize',18);
ylabel('Y(m)','Fontname', 'Times New Roman','FontSize',18);
zlabel('Z(m)','Fontname', 'Times New Roman','FontSize',18);
legend('Total station','Fontname', 'Times New Roman','FontSize',20);
title('Estimated pose with error ellipsoid','Fontname', 'Times New Roman','FontSize',24);



