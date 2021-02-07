%% get_drone_pose: estimate drone pose at body center in local ENU frame with high accuracy (1cm) using total station and onboard IMUs
% By Yue Pan @ ETHZ IGP

clear; clc; close all;
addpath(['..' filesep '..' filesep 'common']);

% sequence for data processing
% drone_data_loader --> get_tran_lt --> get_drone_pose

%%  Load the drone's pose estimated by onboard GNSSINS
log_filename='2021-01-19 15-52-28';  % example log data

project_origin_datenum = 7.381756617086083e+05;     % beginning datenum for example data 
second_in_day = 24 * 3600;

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
track_transaction_id='20210119155250';   %example total station tracking data

track_result_path_base = ['..' filesep '..' filesep 'measure'  filesep 'tps_geocom' filesep 'results'];
load([track_result_path_base filesep track_transaction_id filesep 'meas_polar_' track_transaction_id '.mat'],'meas_polar');
load([track_result_path_base filesep track_transaction_id filesep 'meas_cart_' track_transaction_id '.mat'],'meas_cart');
load([track_result_path_base filesep track_transaction_id filesep 'meas_status_' track_transaction_id '.mat'],'meas_status');
load([track_result_path_base filesep track_transaction_id filesep 'meas_ts_' track_transaction_id '.mat'],'meas_ts');

% use central PC time for sync
tps_ts = meas_ts;

% Get the tracking segements of time of interest (toi)
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

%% Load the camera's position in total station's coordinate system
load(['results' filesep log_filename filesep log_filename '_position_cam.mat'],'cam_pos');
disp('Load camera position successfully');

%% Apply the transformation

tran_mat = [tran_mat_tps2local; 0 0 0 1];
% for drone position
track_points_tps = [meas_cart_toi' ; ones(1,toi_sample_count)]; % [4 * n]
track_points_local = tran_mat * track_points_tps; % [4 * n]
track_points_local = track_points_local(1:3,:)';  % [n * 3]

% for camera position
cam_count = size(cam_pos,1);
cam_tps = [cam_pos'; ones(1,cam_count)]; % [4 * m]
cam_local = tran_mat * cam_tps; % [4 * m]
cam_local = cam_local(1:3,:)';  % [m * 3]

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

% Get data timestamp in the project's reference time system:
ts_project = (tps_ts_toi-project_origin_datenum)*second_in_day;


%% Get the accurate position of the drone 
% use the leverarm
leverarm = [-0.102; 0; 0.19]; % displacement from the prism to the drone's body center [Note: Please change it after your own calibration]
tran_enu2ned = [0 1 0; 1 0 0; 0 0 -1];
z_coordinate_system_gap = 0.0; 

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

% error due to prism displacement
cov_leverarm = diag([5e-4,5e-4,5e-4]).^2;  %prism displacement calibration accuracy:0.5 mm 

% change according to the specification of your own IMU
orientation_error_bias = 1e-3; % Take care of this parameter controling the accuracy of IMU and EKF
orientation_error_rate = 1e-4; % per second

% define total station accuracy
% For Lecia Nova TS-60 with Leica 360 (mini) prism
% when meas_status = 0 (fine tracking)
 std_ang = 0.5 / 3600;  % 0.5 ''
 std_dist_m = 0.6*1e-3; % 0.6 mm
 std_dist_ppm = 1; % 1ppm
 
 
deg2rad = pi/180;
cov_drone_centre_list = cell(toi_sample_count, 1);
cov_prism_tps_list = cell(toi_sample_count, 1);
cov_prism_local_list = cell(toi_sample_count, 1);
cov_prism_drone_list = cell(toi_sample_count, 1);


% Error propagation
% Reference: A tutorial on SE(3) transformation parameterizations and
% on-manifold optimization [Page17, 22-23]
for i=1:toi_sample_count
     R_lb = (eul2rotm(track_points_euler_rad(i,:),'XYZ'))'; % unit:rad
     cov_pose_drone =zeros(6);
     orientation_error_ratio_cur = orientation_error_bias + orientation_error_rate * (tps_ts_toi(i)-tps_ts_toi(1))*second_in_day;
     cov_pose_drone(4,4) = orientation_error_ratio_cur;
     cov_pose_drone(5,5) = orientation_error_ratio_cur;
     cov_pose_drone(6,6) = orientation_error_ratio_cur; 
     cov_pose_drone = cov_pose_drone.^2;
     
     J_mat_prism_drone = getJacobiRp(leverarm, track_points_euler_rad(i,:));
     cov_prism_drone =  J_mat_prism_drone * cov_pose_drone * J_mat_prism_drone' + R_lb * cov_leverarm * R_lb';
     cov_prism_drone_list{i} = cov_prism_drone; % error due to prism displacement after drone orientation transformation
     
     cov_prism_tps = calPointCov(meas_polar_toi(i,1), meas_polar_toi(i,2)*deg2rad, meas_polar_toi(i,3)*deg2rad, std_ang*deg2rad, std_dist_m, std_dist_ppm);
     cov_prism_tps_list{i}=cov_prism_tps; % error due to total station measurement
     
     R_lt = tran_mat_tps2local(1:3,1:3);
     R_lt_euler_rad = rotm2eul(R_lt, 'XYZ'); % unit: rad
     J_mat_prism_local = getJacobiRp(meas_cart_toi(i,:), R_lt_euler_rad);
     cov_prism_local =  J_mat_prism_local * cov_mat_tps2local * J_mat_prism_local' + R_lt * cov_prism_tps * R_lt'; 
     cov_prism_local_list{i} = cov_prism_local; % error due to total station mesurement after geo-referenced transformation
     
     cov_drone_local = cov_prism_drone+cov_prism_local; %[3 x 3] mat  % since they are independent
     cov_drone_centre_list{i} = cov_drone_local;
end
disp('Get the covariance matrix');

%% Preparation for plotting
error_total=zeros(toi_sample_count,1); % drone centre in local frame
error_prism_tps= error_total;  % prism in total station frame
error_prism_local = error_total; % prism in local frame
error_leverarm = error_total;  % prism in body frame
error_prism_drone = error_total; % prism in rotated body frame
error_drone_tps = error_total;  % drone centre in total station frame
error_v=error_total; 
error_h=error_total;
error_x=error_total;
error_y=error_total;

for i=1:toi_sample_count
     error_prism_tps(i) = norm(sqrt(diag(cov_prism_tps_list{i})));  % error due to total station measurement
     error_prism_local(i) =  norm(sqrt(diag(cov_prism_local_list{i})));  % error due to total station mesurement after geo-referenced transformation
     error_leverarm(i) =  norm(sqrt(diag(cov_leverarm)));    % error due to prism displacement
     error_prism_drone(i) = norm(sqrt(diag(cov_prism_drone_list{i})));     % error due to prism displacement after drone orientation transformation
     
     temp_total_error_vector = sqrt(diag(cov_drone_centre_list{i}));
     error_total(i) = norm(temp_total_error_vector); % total error in local frame
     
     error_v(i) = temp_total_error_vector(3); % vertical (or z)
     error_h(i) = norm(temp_total_error_vector(1:2)); % horizontal (x and y)
     error_x(i)= temp_total_error_vector(1); 
     error_y(i)= temp_total_error_vector(2);
end

%% Output pose (write to the dataset)
% Format: Timestamp(s) X(m) Y(m) Z(m) Roll(deg) Pitch(deg) Yaw(deg) Std.X(m) Std.Y(m) Std.Z(m) TrackingStatus
pose_table = [ts_project , track_drone_centre_local, track_points_euler_deg, error_x, error_y, error_v, meas_status_toi];

%% Plot accuracy evaluation

% accuracy in different frame
figure(2)
plot(ts_project, [error_prism_tps, error_prism_local, error_prism_drone, error_total]*100, 'Linewidth', 4);
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',26);
xlabel('Timestamp (s)','Fontname', 'Times New Roman','FontSize',28);
ylabel('Accuracy (cm)','Fontname', 'Times New Roman','FontSize',28);
legend('Prism in total station frame', 'Prism in local frame','Prism in rotated drone body frame','Drone center in local frame','FontSize',26);

% accuracy on different direction
figure(3)
plot(ts_project, [error_x, error_y, error_v, error_total]*100, 'Linewidth', 4);
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',26);
xlabel('Timestamp (s)','Fontname', 'Times New Roman','FontSize',28);
ylabel('Accuracy (cm)','Fontname', 'Times New Roman','FontSize',28);
legend( 'X', 'Y','Z','Total', 'Fontname', 'Times New Roman','FontSize',26);

figure(4)
plot(ts_project, [error_v, error_h, error_total]*100, 'Linewidth', 4);
grid on;
set(gca, 'Fontname', 'Times New Roman','FontSize',26);
xlabel('Timestamp (s)','Fontname', 'Times New Roman','FontSize',28);
ylabel('Accuracy (cm)','Fontname', 'Times New Roman','FontSize',28);
legend('Vertical', 'Horizontal','Total', 'Fontname', 'Times New Roman','FontSize',26);

%% Plot trajectory
disp('Begin Plotting');
figure(1);
set(gcf, 'position', [0 0 1600 900]);
tps_axis_length = 2.0;
tps_linewidth = 3.0;

% plot total station position
plot3(tran_mat_tps2local(1,4),tran_mat_tps2local(2,4),tran_mat_tps2local(3,4)+z_coordinate_system_gap,'o','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',10);
hold on;

% plot camera position
scatter3(cam_local(:,1), cam_local(:,2), cam_local(:,3)+z_coordinate_system_gap, 75, 'om', 'filled');

% plot the pose of the prism (input unit:deg)
% plottraj(track_points_local, meas_status_toi, track_points_euler_deg); 

% plot the pose of the drone's centre (input unit:deg)
% with the error elliposid  
plottraj(track_drone_centre_local,meas_status_toi, track_points_euler_deg, cov_drone_centre_list);

% without the error elliposid
%plottraj(track_drone_centre_local,meas_status_toi, track_points_euler_deg);  

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

% with camera position
%legend('Total station', 'Cameras', 'Drone - start', 'Drone - end','Fontname', 'Times New Roman','FontSize',20, 'Location', 'Best');

% without camera position
legend('Total station', 'Drone - start', 'Drone - end' ,'Fontname', 'Times New Roman','FontSize',20, 'Location', 'Best');

%title('Estimated pose at drone center','Fontname', 'Times New Roman','FontSize',24);
title('Estimated pose with error ellipsoid','Fontname', 'Times New Roman','FontSize',24);

