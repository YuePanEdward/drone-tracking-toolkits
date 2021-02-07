function plottraj(PointList, PointStatus, EulerangleList, CovList,...
startPointColor,endPointColor,circleRadius,lineWidth, ...
axisLength, show_orientation_step)
% plottraj: Plot the trajectory, optionally with the orientation and error ellipsoids 
% By Yue Pan @ ETHZ IGP
% Inputs:
% PointList: 3D point array, default: on
% PointStatus: status of the point measurements (0: ok, 1: warning, 2:error,
% 3: fatal) , only samples with status 0 and 1 would be plotted, default:
% off
% EulerangleList: 3D orientation array (stored as euler angle) unit: deg,
% default: off
% CovList: 3D covariance matrix, unit: m, for plotting error ellipsoids,
% default: off
% startPointColor: the color of starting point, default: green
% endPointColor: the color of end point, default: blue
% circleRadius: the radius of starting and end points , default: 160
% lineWidth: line width of the trajectory, default: 4
% axisLength: length of the orientation axis, default: 1.5 m
% show_orientation_step: show orientation for each K measurement samples
% for making the figure clear, defualt: allow 500 samples with orientation axis
% at most
% No output

% deal with default input
status_on=1;
if(nargin<2)
    status_on=0;
end
attitude_on=1;
if(nargin<3)
    attitude_on=0;
end
cov_on=1;
if(nargin<4)
    cov_on=0;
end
if(nargin<5)
    startPointColor = [0,1,0];
    endPointColor = [0,0,1];
    circleRadius=160;
end
if(nargin<7)
    circleRadius=160;
end
if(nargin<8)
    lineWidth = 3;
end
if(nargin<9)
    % plot the orientation axis
    axisLength = 1.5;   % length of the axis
end

[PointNUM, ~]  = size(PointList);

if(nargin<9)
    show_orientation_step = ceil(PointNUM / 500); % show at most 500 samples with orientation
end
lineWidth_axis= 2.0; 

CovariancePointColor = [0.1, 0.1, 0.1];
sphere_scale = 200;

if(status_on) % only select those putative points
    PointList = PointList(find(PointStatus<2),:); % select only the measurments with the status warning or well 
    if(attitude_on)
          EulerangleList = EulerangleList(find(PointStatus<2),:); % warning or well 
    end
    if(cov_on)
          CovList = CovList(find(PointStatus<2),:); % warning or well 
    end
end  

% the main function body
[PointNUM, ~]  = size(PointList);

scatter3(PointList(1,1),PointList(1,2),PointList(1,3),circleRadius,startPointColor,'filled','marker','o');
hold on;
scatter3(PointList(PointNUM,1),PointList(PointNUM,2),PointList(PointNUM,3),circleRadius,endPointColor,'filled','marker','o');
hold on;

colorStep = (endPointColor-startPointColor)/(PointNUM-1);
for i=1:PointNUM-1
    plot3(PointList(i:i+1,1),PointList(i:i+1,2),PointList(i:i+1,3),'color',colorStep*(i-1)+startPointColor,'linewidth',lineWidth); 
    hold on;
    
     if(cov_on & mod(i, show_orientation_step) ==0 & i <PointNUM-10 ) % Plot error ellipsoids
        ellipsoid(PointList(i,1),PointList(i,2),PointList(i,3),sphere_scale*sqrt(CovList{i}(1,1)),sphere_scale*sqrt(CovList{i}(2,2)),sphere_scale*sqrt(CovList{i}(3,3)),10);
     end
    
    if  (attitude_on & mod(i, show_orientation_step) ==0 ) % Plot orientation aixs
           po = PointList(i,:)';
           cur_euler_angle_rad = pi/180*EulerangleList(i,:);  % deg 2 rad
           R_mat_temp = eul2rotm(cur_euler_angle_rad,'XYZ'); % R_bw
           
           tran_enu2ned = [ 0 1 0; 1 0 0; 0 0 -1];  
           
           po_axis = tran_enu2ned * R_mat_temp' * (axisLength.* eye(3)) + repmat(po,[1,3]);
           px = po_axis(:,1);
           py = po_axis(:,2);
           pz = po_axis(:,3);
           line([po(1); px(1)], [po(2); px(2)], [po(3); px(3)], 'Color', 'r', 'LineWidth', lineWidth_axis); 
           line([po(1); py(1)], [po(2); py(2)], [po(3); py(3)], 'Color', 'g', 'LineWidth', lineWidth_axis); 
           line([po(1); pz(1)], [po(2); pz(2)], [po(3); pz(3)], 'Color', 'b', 'LineWidth', lineWidth_axis); 
    end
end
end