function [objectDetected,distance] = obstacleSensor(sensorAngle,cur_x,...
                        cur_y,sensors,max_x,max_y,Obs_Matrix,visionMatrix)
% Check what the sensor cone "sees"
[x,y,theta]=drawSensorCone(sensorAngle,cur_x+sensors(1),cur_y+sensors(2),0);
% remove last two elements of x and y (as they are only needed for the cone
% plot)
x(end-2:end)=[];
y(end-2:end)=[];

% First build the vision matrix
% l goes from 0 to 1(length of vision cone radius) in 0.01 increments
for l=0:0.01:1
    for i=1:size(x,2)
        theta_current=theta(1,i);
        xdist = l*cos(theta_current);
        ydist = l*sin(theta_current);
        X = round((xdist/0.01) + (cur_y+sensors(2))/0.01 +((max_x/2)/0.01));
        Y = round((ydist/0.01) + (cur_x+sensors(1))/0.01 +((max_y/2)/0.01));
        visionMatrix(Y,X) = 1;
    end
end


% find where 1s appear in the same indices in the vision and obstacle
% matrices
detections=visionMatrix & Obs_Matrix;
[row,col] = find(detections==1);
% if no matches are found then no object has been detected
if isempty(row) || isempty(col)
   objectDetected = 0;
   distance = NaN;
% otherwise object has been detected, find mean distance
else
    objectDetected = 1;
    distance(1) = sqrt((cur_x+(max_y/2) - max(row)*0.01)^2 +...
        (cur_y+(max_x/2) - max(col)*0.01)^2);
    distance(2) = sqrt((cur_x+(max_y/2) - min(row)*0.01)^2 +...
        (cur_y+(max_x/2) - min(col)*0.01)^2);
end
end

