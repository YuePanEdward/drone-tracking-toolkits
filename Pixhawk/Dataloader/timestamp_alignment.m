%timestamp alignment

% IMU: unix usec
% GPS: unix usec, GPStime (GMS:GPS Seconds of Week, GWk:GPS Week) 

% find the gap between time_us and gps_ms (represent in a constant:
% time_gap_us)

% load this time_gap_us to get for other measurements (IMU, etc.) and get
% their timestamp in GPStime