%% get_tan_lt: Get the transformation matrix from total station to local ENU coordinate system
% By Yue Pan @ ETHZ IGP

clear; clc; close all;
format long g;
addpath(['..' filesep '..' filesep 'common']);
mkdir ('results');

%% Mission (transaction) name
log_filename='2021-01-19 15-52-28';  % example data
% at present, you need to save the backsight points' coordinate in total
% station's coordinate system (tps) and WGS 84 system (wgs) mannually 
% to the 'xxxx_backsight_tps.mat' and 'xxxx_backsight_wgs.mat' points

%% Import data and processing
% import the backsight points table to get tps matrix [n * 3]: X(E), Y(N), Z(U), unit: m
load(['results' filesep log_filename filesep log_filename '_backsight_tps.mat'],'tps'); 
disp ('Load TPS coordinates of the prism (East [m], North [m], Up [m]):');
tps

% add the constant between the prisim and the antenna
prism_antenna_gap = 0.055; % 5.5cm [change it in your own set-up]
tps_antenna = tps + [0 0 prism_antenna_gap];
disp ('Get TPS coordinates of the antenna (East [m], North [m], Up [m]):');
tps_antenna

% import the control points table to get wgs matrix [n * 3]: Lat, Lon,Alt, unit: deg , m
load(['results' filesep log_filename filesep log_filename '_backsight_wgs.mat'],'wgs'); 
disp ('Load WGS 84 coordinates (Lat [deg] , Lon [deg], Alt [m]):');
wgs 

% Actually, there's a issue of the altitude, I am not sure what's the attitude
% used in Pixhawk

% Convert to local ENU coordinate system
load(['results' filesep log_filename filesep log_filename '_ref_wgs.mat'],'ref_wgs'); %load the wgs84 coordinate of local reference system

[e,n,u]=blh2enu(wgs(:,1),wgs(:,2),wgs(:,3),ref_wgs(1),ref_wgs(2),ref_wgs(3));
enu_wgs=[e,n,u]; % control points' coordinate in the local coordinate system
disp ('Get local coordinates (East [m], North [m], Up [m]):');
enu_wgs 

% estimate the transformation from tps system to local world coordinate system
% tran_status=1;
tran_status = 0; % now we only enable the 4dof tran. para. estimation by least square (assume the total station has been leveled)
if (size(tps_antenna,1) > 3 & tran_status ==1)
     [tran_mat_tps2local, cov_mat_tps2local, tran_status] = coor_sys_tran_svd(tps_antenna, enu_wgs); 
     % tran_status=1;
end
if (size(tps_antenna,1) <=3  | tran_status==0)
     %[tran_mat_tps2local, cov_mat_tps2local , tran_status] = coor_sys_tran_4dof(tps_antenna(1:2,:), enu_wgs(1:2,:));  % also try to use the 4dof estimation (we fix roll and pitch by doing the leveling quite well)
     [tran_mat_tps2local, cov_mat_tps2local, tran_status] = coor_sys_tran_4dof(tps_antenna, enu_wgs); 
end


disp ('Transformation matrix from total station to local reference coordinate system:');
tran_mat_tps2local
save(['results' filesep log_filename filesep log_filename '_tran_mat_tps2local.mat'],'tran_mat_tps2local'); 

disp('Covariance of the estimated transformation:')
cov_mat_tps2local 
% sequence: (tx,ty,tz,roll,pitch,yaw) 
save(['results' filesep log_filename filesep log_filename '_cov_mat_tps2local.mat'],'cov_mat_tps2local'); 