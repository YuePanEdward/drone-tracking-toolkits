% LSQ Linear

% Measurements
m = [1.992 3.018 3.997 5.001 0.997 0.983]';

% Distance over measurements
d = [600 600 300 600 500 400]';

% Standard deviations
StdDev = 0.015 * sqrt(d./1000);

% Constants
constants = [-10 15 0 0 -15 0]';

% Design matrix
A = [1 0 0; -1 0 0; -1 1 0; -1 0 1; 0 1 0; 0 -1 1];


% No need to change the following part

% Reduced measurements
mr = m - constants;

% Variance-covariance matrix
Vm = diag(StdDev.^2);

% Normal matrix
norm = A' * inv(Vm) * A;

% Estimated x
xHat = inv(norm) * A' * inv(Vm) * mr;

% Adjusted reduced measurements
mr_adj = A * xHat;

% Adjusted measurements
m_adj = mr_adj + constants;

% Corrections
corrections = m_adj - m;

% Variance-covariance matrix of estimated x
VxHat = inv(norm);

% Variance-covariance matrix of adjusted measurements
Vm_adj = A * VxHat * A';

% Variance-covariance matrix of corrections
Vcorrections = Vm - Vm_adj;

% Standard deviations of adjusted measurements
StdDev_m_adj = diag(Vm_adj).^0.5;

% Standard deviations of adjusted corrections
StdDev_corrections = diag(Vcorrections).^0.5;