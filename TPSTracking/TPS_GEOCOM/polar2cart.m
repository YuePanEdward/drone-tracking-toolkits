function [X,Y,Z] = polar2cart(D, Hr, V)
%polar2cart Convert from polar to cartesian coordinate system
%   Detailed explanation goes here
X=D*cosd(V)*cosd(Hr);
Y=D*cosd(V)*sind(Hr);
Z=D*sind(V);

end

