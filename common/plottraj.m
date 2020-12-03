function plottraj(PointList, PointStatus, EulerangleList, ...
startPointColor,endPointColor,circleRadius,lineWidth, ...
axisLength, show_attitude_step)
%plottraj 
%PointList: 3D point array
%PointStatus: status of the point measurements (0: ok, 1: warning, 2:error, 3: fatal) 
%EulerangleList: 3D attitude array (stored as euler angle) unit: deg
%startPointColor: the color of starting point
%endPointColor: the color of end point
%circleRadius: the tadius of starting and end points
% Below is some default params
 
% deal with default input
status_on=1;
if(nargin<2)
    status_on=0;
end
attitude_on=1;
if(nargin<3)
    attitude_on=0;
end
if(nargin<4)
    startPointColor = [1,0,0];
    endPointColor = [0,1,0];
    circleRadius=80;
end
if(nargin<6)
    circleRadius=80;
end
if(nargin<7)
    lineWidth = 3;
end
if(nargin<8)
    % plot the attitude
    axisLength = 0.25; 
end

[PointNUM, ~]  = size(PointList);

if(nargin<9)
    show_attitude_step = ceil(PointNUM / 500);
end
lineWidth_axis= 0.8;
 
if(status_on) % only select those putative points
    PointList = PointList(find(PointStatus<2),:); % warning or well 
    if(attitude_on)
          EulerangleList = EulerangleList(find(PointStatus<2),:); % warning or well 
    end
end  

% the main function body
[PointNUM, ~]  = size(PointList);

scatter3(PointList(1,1),PointList(1,2),PointList(1,3),circleRadius,startPointColor,'filled');
hold on;
scatter3(PointList(PointNUM,1),PointList(PointNUM,2),PointList(PointNUM,3),circleRadius,endPointColor,'filled');
hold on;

colorStep = (endPointColor-startPointColor)/(PointNUM-1);
for i=1:PointNUM-1
    plot3(PointList(i:i+1,1),PointList(i:i+1,2),PointList(i:i+1,3),'color',colorStep*(i-1)+startPointColor,'linewidth',lineWidth); 
    hold on;
    
    if  (attitude_on & mod(i, show_attitude_step) ==0 )
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