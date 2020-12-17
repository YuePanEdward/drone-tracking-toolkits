function [Cov_xyz] = calPointCov(rou, theta, alpha, std_ang, std_dist_m, std_dist_ppm)
%CALPOINTCOV calculate the x,y,z covariance matrix of one group of TPS measurements 
%   inputs:
%   - rou : distance measurement (unit: m)
%   - theta: horizontal angle measurement (unit: deg)  
%   - alpha: vertical angle measurement (unit: deg)  
%   - std_ang: measurement accuracy (standard deviation) of horizontal and
%   vertical angle measurements (unit: deg)
%   - std_dist_m: basic measurement accuracy (standard deviation) of distance
%   (unit:m)
%   - std_dist_ppm: scalar measurement accuracy (standard deviation) of distance
%   (unit: ppm, 1mm/1km, 1e-6) 
%   output:
%   Cov_xyz: [3 x 3] covariance matrix of x,y,z coordinate


% Assign Jacobian matrix [3 x 3]
J_mat = [sind(theta) * cosd(alpha),    rou*cosd(theta)*cosd(alpha),   -rou*sind(theta)*sind(alpha);
               cosd(theta) * cosd(alpha),   -rou*sind(theta)*cosd(alpha),   -rou*cosd(theta)*sind(alpha);
               sind(alpha) ,                         0,                                               rou*cosd(alpha)                  ];
           
Cov_rou_theta_alpha = diag([std_dist_m + std_dist_ppm * 1e-6 * rou, std_ang, std_ang]).^2;

Cov_xyz =  J_mat * Cov_rou_theta_alpha * J_mat';

end

