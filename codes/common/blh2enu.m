function [e, n, u] = blh2enu(b, l, h, ref_b, ref_l, ref_h)
% inputs:
% b,l,h are K * 1 vectors (ECEF coordinates of K points) , unit: deg, m
% ref_b,ref_l,ref_h are single number(ECEF coordinate of reference
% station), unit: deg,m
% outputs:
% e,n,u are K * 1 vectors (ENU coordinates of K points)
% implemented based on Standford GPS toolbox


tranmat_xyz2enu = findxyz2enu(deg2rad(ref_b), deg2rad(ref_l)); % input(rad)

[x, y, z] = blh2xyz(b, l, h);  %input(deg)
[ref_x, ref_y, ref_z] = blh2xyz(ref_b, ref_l, ref_h); %input(deg) 

pos_relative_xyz=[x-ref_x,y-ref_y,z-ref_z];       % k*3 mat
pos_enu=(tranmat_xyz2enu * pos_relative_xyz')';   % k*3 mat

e=pos_enu(:,1);
n=pos_enu(:,2);
u=pos_enu(:,3);
