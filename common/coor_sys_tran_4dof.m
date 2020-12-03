function [tran_mat, status] = coor_sys_tran_4dof(source, target)
% coor_sys_tran Function estimate the 4DOF transformation
% from two point cloud (source to target) with pre-determined correspondence
% the roll and pitch angle are fixed to 0
% input: source [2 * 3] vector, target [2 * 3] vector
% output: tran_mat [3 * 4] matrix, status: 0 or 1

tz= mean(target(:,3)-source(:,3));
px_1 = source(1,1); py_1 = source(1,2);
px_2 = source(2,1); py_2 = source(2,2);
qx_1 = target(1,1); qy_1 = target(1,2);
qx_2 = target(2,1); qy_2 = target(2,2);

theta_0 = pi/4; % an initial guess
thre = 1e-6; 

while (1)

A_mat = [ -sin(theta_0) * px_1 - cos(theta_0) * py_1 , 1 , 0;...
                  cos(theta_0) * px_1 - sin(theta_0) * py_1 , 0 , 1;...
                 -sin(theta_0) * px_2 - cos(theta_0) * py_2 , 1 , 0;...
                  cos(theta_0) * px_2 - sin(theta_0) * py_2,  0 , 1];
              
b_vec = [qx_1  - cos(theta_0) * px_1 + sin(theta_0) * py_1;...
               qy_1  - sin(theta_0)  * px_1 -  cos(theta_0)* py_1;...
               qx_2  - cos(theta_0) * px_2 + sin(theta_0) * py_2;...
               qy_2  - sin(theta_0)  * px_2 -  cos(theta_0)* py_2];
           
x_vec = inv(A_mat'*A_mat) * A_mat' * b_vec;
d_theta= x_vec(1);

theta_0 = theta_0 + d_theta;

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
               
p_tran_1 = R_mat *  source(1,:)' + t_vec;             
d_1 = norm(target(1,:)' -  p_tran_1);
p_tran_2 = R_mat *  source(2,:)' + t_vec;             
d_2 = norm(target(2,:)' -  p_tran_2);

disp('transform residual (m):');
d_1
d_2

status = 1;
d_thre = 0.05; % unit:m 
if((d_1+d_2)/2 > d_thre)
    status = 0; % problematic transformation
end 

end
