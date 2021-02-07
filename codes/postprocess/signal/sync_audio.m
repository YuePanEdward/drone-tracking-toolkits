function [sync_coeff] = sync_audio(ta1,ta2,tb1,tb2)
%SYNC_AUDIO calculate the synchronization coefficients
% By Yue Pan @ ETHZ IGP
%   a is the time series waiting for synchronization
%    b is the referenced time series
%    [input] ta1,ta2,tb1,tb2: 2 pair of timestamp in both a and b time
%    series
%    [output] sync_coeff: (k, dt) 
%    k is the scale
%    dt is the time shift
%    we have, tbi = tai * k + dt, for any i
 
k = (tb2-tb1)/(ta2-ta1);
dt = (tb1*ta2 - ta1 * tb2)/(ta2-ta1);

sync_coeff(1) =k;
sync_coeff(2) = dt;
end

