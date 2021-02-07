function [local_datenum] = gps2local(gps_week, gps_sow, utc_gap)

% SYNTAX:
%  local_datenum = gps2local(gps_week, gps_sow, utc_gap);
%
% INPUT:
%   gps_week = GPS week
%   gps_sow  = GPS seconds of week
%   utc_gap = local_time - utc_time (in hour), example: [Zurich UTC+2/+1]
%  
% OUTPUT:
%   local_datenum = local time in datenum format (unit: day)
%
%
% DESCRIPTION:
%   Conversion from GPS time to local time, considering the leap second and
%   UTC gap

gps_start_datenum = 723186; %This is datenum([1980,1,6,0,0,0])
gps_datenum = gps_start_datenum + gps_week*7 + gps_sow/86400;

[utc_datenum, leapsec] = gps2utc(gps_datenum);

local_datenum = utc_datenum + utc_gap/24;

end