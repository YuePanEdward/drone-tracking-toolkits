function J_mat = getJacobiRp(pt, eulerangle)
%GETJACOBIRP Summary of this function goes here
%   Detailed explanation goes here
% input: 
% - pt [1 x 3]: point coordinate
% - eulerangle [1 x 3]: roll, pitch, yaw of the rotation, (unit: rad)
% output: the Jacobi matrix [ 3 x 6 ]

% Reference: A tutorial on SE(3) transformation parameterizations and
% on-manifold optimization (Page 23)

ax = pt(1); ay = pt(2); az = pt(3);
roll= eulerangle(1); pitch = eulerangle(2); yaw = eulerangle(3);

j14 = -ax*sin(yaw)*cos(pitch)+ ay*(-sin(yaw)*sin(pitch)*sin(roll)-cos(yaw)*cos(roll))+az*(-sin(yaw)*sin(pitch)*cos(roll)+cos(yaw)*sin(pitch));
j15 = -ax*cos(yaw)*sin(pitch) +ay*(cos(yaw)*cos(pitch)*sin(roll))+az*(cos(yaw)*cos(pitch)*cos(roll));
j16 = ay*(cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll))+az*(-cos(yaw)*sin(pitch)*sin(roll)+sin(yaw)*cos(roll));
j24 = ax*cos(yaw)*cos(pitch) + ay*(cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll))+az*(cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll));
j25 = -ax*sin(yaw)*sin(pitch) + ay*(sin(yaw)*cos(pitch)*sin(roll))+az*sin(yaw)*cos(pitch)*cos(roll);
j26 = ay*(sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll))+az*(-sin(yaw)*sin(pitch)*sin(roll)-cos(yaw)*cos(roll));
j34 = 0;
j35 = -ax*cos(pitch) - ay*sin(pitch)*sin(roll) - az*sin(pitch)*cos(roll);
j36 = ay*cos(pitch)*cos(roll) - az*cos(pitch)*sin(roll);

J_mat_left = eye(3);
J_mat_right = [j14, j15, j16; j24, j25, j26; j34, j35, j36];
J_mat = [J_mat_left, J_mat_right];

end

