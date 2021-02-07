function [e, n, u] = xyz2enu(x, y, z, ref_x, ref_y, ref_z)
% inputs:
% x,y,z are K * 1 vectors (ECEF coordinates of K points)
% ref_x,ref_y,ref_z are single number(ECEF coordinate of reference station)
% outputs:
% e,n,u are K * 1 vectors (ENU coordinates of K points)
% implemented based on Standford GPS toolbox

[ref_b, ref_l] = xyz2blh(ref_x,ref_y,ref_z); %rad
tranmat_xyz2enu = findxyz2enu(ref_b, ref_l);

pos_relative_xyz=[x-ref_x,y-ref_y,z-ref_z];       % k*3 mat
pos_enu=(tranmat_xyz2enu * pos_relative_xyz')';   % k*3 mat

e=pos_enu(:,1);
n=pos_enu(:,2);
u=pos_enu(:,3);




