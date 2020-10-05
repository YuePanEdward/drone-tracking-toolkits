% Least Squares Non-Linear
% Example in Section 8 Lecture

% Measurements
m = [250.802 223.616 234.311 deg2rad(85.424) deg2rad(26.571) deg2rad(320.190)]';

% Standard deviations
StdDev = [0.015 0.015 0.015 deg2rad(10/3600) deg2rad(10/3600) deg2rad(10/3600)];

% Variance-covariance matrix
Vm = diag(StdDev.^2);

% Constants
E1 = 100;
N1 = 100;

% Observation equations
syms E2 E3 N2 N3 d1 d2 d3 Theta1 Theta2 Theta3;
d1 = sqrt((E2 - E1)^2 + (N2 - N1)^2);
d2 = sqrt((E3 - E1)^2 + (N3 - N1)^2);
d3 = sqrt((E3 - E2)^2 + (N3 - N2)^2);
Theta1 = atan((E2 - E1)/(N2 - N1));
Theta2 = atan((E3 - E1)/(N3 - N1));
Theta3 = atan((E3 - E2)/(N3 - N2));

% Partial derivatives
d_d1_E2 = diff(d1, E2);
d_d1_N2 = diff(d1, N2);
d_d1_E3 = diff(d1, E3);
d_d1_N3 = diff(d1, N3);

d_d2_E2 = diff(d2, E2);
d_d2_N2 = diff(d2, N2);
d_d2_E3 = diff(d2, E3);
d_d2_N3 = diff(d2, N3);

d_d3_E2 = diff(d3, E2);
d_d3_N2 = diff(d3, N2);
d_d3_E3 = diff(d3, E3);
d_d3_N3 = diff(d3, N3);

d_Theta1_E2 = diff(Theta1, E2);
d_Theta1_N2 = diff(Theta1, N2);
d_Theta1_E3 = diff(Theta1, E3);
d_Theta1_N3 = diff(Theta1, N3);

d_Theta2_E2 = diff(Theta2, E2);
d_Theta2_N2 = diff(Theta2, N2);
d_Theta2_E3 = diff(Theta2, E3);
d_Theta2_N3 = diff(Theta2, N3);

d_Theta3_E2 = diff(Theta3, E2);
d_Theta3_N2 = diff(Theta3, N2);
d_Theta3_E3 = diff(Theta3, E3);
d_Theta3_N3 = diff(Theta3, N3);

% For correct bearings
Theta1 = mod(-atan2(N2 - N1, E2 - E1) + pi/2, 2*pi);
Theta2 = mod(-atan2(N3 - N1, E3 - E1) + pi/2, 2*pi);
Theta3 = mod(-atan2(N3 - N2, E3 - E2) + pi/2, 2*pi);

% Initial guess
E2_est = m(1, 1) * sin(m(4, 1)) + E1;
N2_est = m(1, 1) * cos(m(4, 1)) + N1;
E3_est = m(2, 1) * sin(m(5, 1)) + E1;
N3_est = m(2, 1) * cos(m(5, 1)) + N1;

% Iterate until the corrections are within 0.1 mm
xDelta = [100 100 100 100]';
req = 0.0001;

% Iteration number
i = 0;

% Iteration
while abs(xDelta(1, 1)) >= req || abs(xDelta(2, 1)) >= req || abs(xDelta(3, 1)) >= req || abs(xDelta(4, 1)) >= req
    % Iteration number
    i = i + 1;
    disp("Iteration No." + i);
    
    % Jacobian matrix
    J = [vpa(subs(d_d1_E2, {E2, N2}, {E2_est, N2_est})) ...
         vpa(subs(d_d1_N2, {E2, N2}, {E2_est, N2_est})) d_d1_E3 d_d1_N3;
         d_d2_E2 d_d2_N2 vpa(subs(d_d2_E3, {E3, N3}, {E3_est, N3_est})) ...
         vpa(subs(d_d2_N3, {E3, N3}, {E3_est, N3_est}));
         vpa(subs(d_d3_E2, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est})) ...
         vpa(subs(d_d3_N2, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est})) ...
         vpa(subs(d_d3_E3, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est})) ...
         vpa(subs(d_d3_N3, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est}));
         vpa(subs(d_Theta1_E2, {E2, N2}, {E2_est, N2_est})) ...
         vpa(subs(d_Theta1_N2, {E2, N2}, {E2_est, N2_est})) d_Theta1_E3 d_Theta1_N3;
         d_Theta2_E2 d_Theta2_N2 vpa(subs(d_Theta2_E3, {E3, N3}, {E3_est, N3_est})) ...
         vpa(subs(d_Theta2_N3, {E3, N3}, {E3_est, N3_est}));
         vpa(subs(d_Theta3_E2, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est})) ...
         vpa(subs(d_Theta3_N2, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est})) ...
         vpa(subs(d_Theta3_E3, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est})) ...
         vpa(subs(d_Theta3_N3, {E2, N2, E3, N3}, {E2_est, N2_est, E3_est, N3_est}))];
    
    % Computed measurements
    mc = [vpa(subs(d1, {E2, N2}, {E2_est, N2_est}));
          vpa(subs(d2, {E3, N3}, {E3_est, N3_est}));
          vpa(subs(d3, {E2, E3, N2, N3}, {E2_est, E3_est, N2_est, N3_est}));
          vpa(subs(Theta1, {E2, N2}, {E2_est, N2_est}));
          vpa(subs(Theta2, {E3, N3}, {E3_est, N3_est}));
          vpa(subs(Theta3, {E2, E3, N2, N3}, {E2_est, E3_est, N2_est, N3_est}))];
    
    % Reduced measurements (observed - computed)
    mr = vpa(m - mc)
    
    % Normal matrix
    norm = J' * inv(Vm) * J;
    
    % Corrections ([dE2; dN2; dE3; dN3])
    xDelta = inv(norm) * J' * inv(Vm) * mr
    
    % Apply the corrections before next iteration
    E2_est = E2_est + xDelta(1, 1);
    N2_est = N2_est + xDelta(2, 1);
    E3_est = E3_est + xDelta(3, 1);
    N3_est = N3_est + xDelta(4, 1);
end

% Estimated x ([E2 N2 E3 N3])
x = [E2_est N2_est E3_est N3_est]'

% Variance-covariance matrix of estimated x
Vx = inv(norm);

% Standard deviations of the solution
StdDev_x = diag(Vx).^0.5

% Variance-covariance matrix of the residuals
residuals = mr
Vresiduals = Vm - J * Vx * J';