function [X,Y,Z] = polar2cart(D, Hr, V)
%polar2cart Convert from polar to cartesian coordinate system
%   Detailed explanation goes here
X=D*cosd(V)*sind(Hr); 
Y=D*cosd(V)*cosd(Hr);
Z=D*sind(V);

% Hr begin clockwise from the azimuth (north) direction (y+) 
% X means East direction
% Y means North direction
% Z means Up direction
% XYZ (ENU)

end

