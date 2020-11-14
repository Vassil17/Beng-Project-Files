function [objectDetected,distance] = obstacleSensor(cur_psi,cur_x,cur_y,sensors,max_x,max_y,Obs_Matrix,visionMatrix)
% Check what the sensor cone "sees"
[x,y,theta]=drawSensorCone(cur_psi,cur_x+sensors(1),cur_y+sensors(2),0);
% remove last two elements of x and y (as they are only needed for the cone
% plot)
x(end-2:end)=[];
y(end-2:end)=[];

% First build the vision matrix
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
% find where 1s appear in the same indeces in the vision and obstacle
% matrices
detections=visionMatrix & Obs_Matrix;
[row,col] = find(detections==1);
if isempty(row) || isempty(col)
   objectDetected = 0;
else
    objectDetected = 1;
    distance = sqrt((cur_x+(max_y/2) - mean(row)*0.01)^2 +...
        (cur_y+(max_x/2) - mean(col)*0.01)^2);
end


end

