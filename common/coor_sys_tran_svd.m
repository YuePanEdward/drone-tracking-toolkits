function [tran_mat, status] = coor_sys_tran_svd(source, target)
% coor_sys_tran Function estimate the 6DOF rigid body transformation
% from two point cloud (source to target) with pre-determined correspondence
% Using singular value decomposition (SVD) closed-form solution
% refer to [Horn, 1987]
% input: source [n * 3] vector, target [n * 3] vector
% output: tran_mat [3 * 4] matrix, status 0 or 1 (means rigid
% transformation or not)

status =1;

source_grav_cent=mean(source);
target_grav_cent=mean(target);
source_degrav=source-source_grav_cent;   % source (p)
target_degrav=target-target_grav_cent;   % target (q)
[U,S,V]=svd(source_degrav'*target_degrav); % apply SVD  
R_mat=V*U'; % Rotation matrix (source to target) R_os  [3*3]
t_vec=target_grav_cent'-(R_mat*source_grav_cent'); % translation vector (source to target) t_os [3*1]

tran_mat = [R_mat, t_vec]; % [3*4]

if(det(R_mat) < 0) % det(R_mat)==-1, mirror reverse transformation instead of rotation
     status = 0;
end

end