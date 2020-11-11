function plottraj(PointList, startPointColor,endPointColor,circleRadius)
%plottraj 
%PointList ：3D point array
%startPointColor: the color of starting point
%endPointColor: the color of end point
%circleRadius: the tadius of starting and end points
% Below is some default params
 
if(nargin<2)
    startPointColor = [1,0,0];
    endPointColor = [0,1,0];
    circleRadius=60;
elseif(nargin<4)
    circleRadius=60;
end
 
% the main function body
[PointNUM, ~]  = size(PointList);

scatter3(PointList(1,1),PointList(1,2),PointList(1,3),circleRadius,startPointColor,'filled');
hold on;
scatter3(PointList(PointNUM,1),PointList(PointNUM,2),PointList(PointNUM,3),circleRadius,endPointColor,'filled');
hold on;

colorStep = (endPointColor-startPointColor)/(PointNUM-1);
for i=1:PointNUM-1
    plot3(PointList(i:i+1,1),PointList(i:i+1,2),PointList(i:i+1,3),'color',colorStep*(i-1)+startPointColor); 
    hold on;
end

end