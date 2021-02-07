function [tran_mat, Cov_mat, status] = coor_sys_tran_4dof(source, target)
% coor_sys_tran Function estimate the 4DOF transformation
% By Yue Pan @ ETHZ IGP
% from two point cloud (source to target) with pre-determined correspondence
% the roll and pitch angle are fixed to 0
% input: source [n * 3] vector, target [n * 3] vector
% output: tran_mat [3 * 4] matrix, covariance matrix of the transforamtion estimation [6 * 6] matrix, status: 0 or 1

count_correspondence = size(source,1);

tz= mean(target(:,3)-source(:,3));

theta_0 = pi/4; % an initial guess
thre = 1e-12; 

iter_count =0;
while (1)

A_mat = [];
b_vec = [];
for i=1: count_correspondence
      
      A_mat = [A_mat;  
                     -sin(theta_0) * source(i,1) - cos(theta_0) * source(i,2) , 1 , 0;
                      cos(theta_0) * source(i,1)  - sin(theta_0) * source(i,2)  , 0 , 1];
       
      b_vec = [b_vec;
                     target(i,1)  - cos(theta_0) * source(i,1) + sin(theta_0) * source(i,2);
                     target(i,2)  - sin(theta_0)  * source(i,1) -  cos(theta_0)*source(i,2)];
                   
end
           
x_vec = inv(A_mat'*A_mat) * A_mat' * b_vec;
d_theta= x_vec(1);

theta_0 = theta_0 + d_theta;
iter_count = iter_count  + 1;

if (abs(d_theta) < thre)
     break;
end
  
end           

tx = x_vec(2);
ty = x_vec(3);
theta = theta_0;

R_mat = [cos(theta), -sin(theta), 0;...
                sin(theta), cos(theta), 0;...
                0,0,1];
t_vec = [tx; ty; tz];

tran_mat = [R_mat,  t_vec];

disp('transform residual (m):');
res_norm = zeros(count_correspondence,1);
for i=1:count_correspondence
    p_tran_temp = R_mat *  source(i,:)' + t_vec;             
    res_norm(i) = norm(target(i,:)' -  p_tran_temp);
end
res_norm

status = 1;
d_thre = 0.05; % unit:m 
if(mean(res_norm) > d_thre)
    status = 0; % problematic transformation
    disp('Too large residual after tansformation');
end 

% covariance matrix of the estimation:
% Change it according to the specification of your instruments
% Trimble R8 GNSS (With SwiPos RTK service) accuracy: 8 mm Horizontal / 15 mm Vertical
rtk_std = [8e-3,8e-3,15e-3]; % unit: m
leveling_std = [1.0,1.0]; % roll, pitch, unit: angular second
%(make sure the leveling angle is as small as 1 second  by tuning the leveling bubble to locate it at the centre of the circle)
leveling_std = leveling_std/(180/pi*3600); % unit: rad 

count_correspondence = 8; % comment this line , use the actual number of back-sight points instead

res_vec = A_mat*x_vec - b_vec;
sigma2_posterior = 1/(2*count_correspondence-3) * res_vec' * res_vec;
Cov_x = sigma2_posterior*pinv(A_mat' * A_mat); % yaw, tx, ty
std2_tz = 1/count_correspondence * (rtk_std(3)^2);

Cov_mat = zeros(6);
Cov_mat(3:5, 3:5) = Cov_x;
Cov_mat(6,6) = std2_tz;
Cov_mat(1,1)=leveling_std(1)^2;
Cov_mat(2,2)=leveling_std(2)^2;

% convert from the sequence:  from (roll, pitch, yaw, x, y, z) to  (x,y,z,roll,pitch,yaw) 
Cov_mat_temp = Cov_mat;
Cov_mat_temp(1:3,1:3) = Cov_mat(4:6,4:6);
Cov_mat_temp(4:6,4:6) = Cov_mat(1:3,1:3);
Cov_mat_temp(1:3,4:6) = Cov_mat(4:6,1:3);
Cov_mat_temp(4:6,1:3) = Cov_mat(1:3,4:6);
Cov_mat = Cov_mat_temp; 

end
