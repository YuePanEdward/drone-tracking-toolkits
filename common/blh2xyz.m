function [X, Y, Z] = blh2xyz(lat, lon, alt)
%
% function blh2xyz calculate geocentric coordinates based on geodetic
% coordinates and parameters of reference ellipsoid
% inputs:
% lat        point's latitude, input as decimal degrees
% lon        point's longitude, input as decimal degrees
% alt        point's altitude, input in metres
% outputs:
% X, Y, Z   geocentric coordinates of input point, output in metres

%% constants:
a=6378137;       % semi major axis of reference ellipsoid, input in metres
e2=0.0066943799013; % eccentricity of reference ellipsoid

%% Algorithm

N = a./((1-e2.*((sind(lon)).^2)).^(1/2));

X = ((N+alt).*cosd(lat).*cosd(lon));
Y = ((N+alt).*cosd(lat).*sind(lon));
Z = (((N.*(1-e2))+alt).*sind(lat));

end