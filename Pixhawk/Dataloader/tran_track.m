%%
clear; clc; close all;
addpath(['..' filesep '..' filesep 'common']);

%%  Load the drone's pose estimated by onboard GNSSINS
%log_filename='2020-11-23 17-30-31';  %test_7
log_filename='2020-11-25 17-21-24';  %test_8

load(['results' filesep log_filename filesep log_filename '_position_ekf.mat'],'track_position_ekf'); % E, N, U
load(['results' filesep log_filename filesep log_filename '_attitude_ekf.mat'],'track_attitude_ekf'); % roll, pitch, yaw
load(['results' filesep log_filename filesep log_filename '_gpsts_ekf.mat'],'track_time_gps_sow_ekf');  % timestamp

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
load([track_result_path_base filesep track_transaction_id filesep 'meas_cart_' track_transaction_id '.mat'],'meas_cart');
load([track_result_path_base filesep track_transaction_id filesep 'meas_status_' track_transaction_id '.mat'],'meas_status');
load([track_result_path_base filesep track_transaction_id filesep 'meas_ts_' track_transaction_id '.mat'],'meas_ts');
load([track_result_path_base filesep track_transaction_id filesep 'pc_ts_' track_transaction_id '.mat'],'pc_ts');

pc_tps_shift = 0.1; % unit: s

tps_ts = pc_ts (:,1:6);
tps_ts(:,6) = tps_ts(:,6) + 0.001 * pc_ts (:,7) + pc_tps_shift;  % convert to [Year Month Day Hour Minute Second.]
[meas_gps_week, meas_gps_sow, meas_gps_dow] = local2gps(tps_ts, 1); % 1 means UTC+1

% Get the tracking segements of time of interest (toi)
index_toi_tps_track = find(meas_gps_sow>track_time_gps_sow_ekf(1) &  meas_gps_sow<track_time_gps_sow_ekf(end) & meas_status<2);
meas_cart_toi = meas_cart(index_toi_tps_track, :);
meas_status_toi = meas_status(index_toi_tps_track, :);
meas_gps_sow_toi = meas_gps_sow(index_toi_tps_track, :);
toi_sample_count = size(meas_cart_toi,1);

disp('Load the total station tracking position of the prism successfully');

%% Load the transformation matrix from total station to local reference coordinate system
load(['results' filesep log_filename filesep log_filename '_tran_mat_tps2local.mat'],'tran_mat_tps2local');

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
    cur_tps_ts = meas_gps_sow_toi(i);
    % TODO: 
    % find the adjacent quaternions of each tracked position according to the GPS timestamp for interpolation 
    % use slerp
    delta_t = track_time_gps_sow_ekf - cur_tps_ts;
    indices_pre = find(delta_t<=0);
    index_pre = indices_pre(end);
    indices_post = find(delta_t>0);
    index_post = indices_post(1);
    interpolate_ratio = (cur_tps_ts - track_time_gps_sow_ekf(index_pre)) / (track_time_gps_sow_ekf(index_post) - track_time_gps_sow_ekf(index_pre)); 
    interpolate_quat = slerp(quat_ekf(index_pre),quat_ekf(index_post),interpolate_ratio); 
    interpolate_euler = quat2eul(interpolate_quat, 'XYZ'); % unit:rad
    track_points_euler_rad(i,:) = interpolate_euler; % unit:rad
end
disp('Conduct the sychronization successfully');

%% Get the accurate position of the drone 
% use the leverarm
leverarm = [-0.102; 0; 0.19];
tran_enu2ned = [0 1 0; 1 0 0; 0 0 -1];

track_drone_centre_local = zeros(toi_sample_count,3);
for i=1:toi_sample_count
     R_wb = (eul2rotm(track_points_euler_rad(i,:),'XYZ'))'; % unit:rad
     track_drone_centre_local(i,:) =  (track_points_local(i,:)' -  tran_enu2ned * R_wb * leverarm)'; 
end
track_points_euler_deg = 180/pi *track_points_euler_rad;

disp('Get the position of the drone centre');


%% Plot 
disp('Begin Plotting');
figure(1);
tps_axis_length = 0.6;
tps_linewidth = 3;
% plot total station position
plot3(tran_mat_tps2local(1,4),tran_mat_tps2local(2,4),tran_mat_tps2local(3,4),'o','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',12);
hold on;
% plot the pose of the prism (input unit:deg)
% plottraj(track_points_local, meas_status_toi, track_points_euler_deg); 
% plot the pose of the drone's centre (input unit:deg)
plottraj(track_drone_centre_local,meas_status_toi, track_points_euler_deg); 
% plot the pose of the drone's centre estimated the drone's ekf
% status_vec= zeros(size(track_position_ekf,1),1);
% plottraj(track_position_ekf, status_vec, track_attitude_ekf); 
% plot total station axis
tpso = tran_mat_tps2local(:,4);
po_axis = tran_mat_tps2local(:,1:3) * (tps_axis_length.* eye(3)) + repmat(tpso,[1,3]);
tpsax = po_axis(:,1); tpsay = po_axis(:,2); tpsaz = po_axis(:,3);
line([tpso(1); tpsax(1)], [tpso(2); tpsax(2)], [tpso(3); tpsax(3)], 'Color', 'r', 'LineWidth', tps_linewidth); 
line([tpso(1); tpsay(1)], [tpso(2); tpsay(2)], [tpso(3); tpsay(3)], 'Color', 'g', 'LineWidth', tps_linewidth); 
line([tpso(1); tpsaz(1)], [tpso(2); tpsaz(2)], [tpso(3); tpsaz(3)], 'Color', 'b', 'LineWidth', tps_linewidth); 
grid on;
axis equal;
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
xlabel('X(m)','Fontname', 'Times New Roman','FontSize',14);
ylabel('Y(m)','Fontname', 'Times New Roman','FontSize',14);
zlabel('Z(m)','Fontname', 'Times New Roman','FontSize',14);
legend('Total station', 'Start', 'End','Fontname', 'Times New Roman','FontSize',10);
title('Estimated Pose of the drone','Fontname', 'Times New Roman','FontSize',16);
