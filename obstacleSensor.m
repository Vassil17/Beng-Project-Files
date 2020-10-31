function [objectDetected,distance_min] = obstacleSensor(cur_psi,cur_x,cur_y,max_x,max_y,Obs_Matrix)
% Check what the sensor cone "sees"
[x,y,theta]=drawSensorCone(cur_psi,cur_x,cur_y,0);
% remove last two elements of x and y (as they are only needed for the cone
% plot)
x(end-2:end)=[];
y(end-2:end)=[];
% counter for number of detections
k=0;
% check each point of the matrix
for l=1:-0.01:0.01
    for i=1:size(x,2)
        theta_current=theta(1,i);
        xdist = l*cos(theta_current);
        ydist = l*sin(theta_current);
        sensorPosX = round((xdist/0.01) + cur_y/0.01 +((max_x/2)/0.01));
        sensorPosY = round((ydist/0.01) + cur_x/0.01 +((max_y/2)/0.01));
        % check if sensor is out of range 
        if sensorPosX < 1 || sensorPosX > max_x/0.01...
                || sensorPosY < 1 || sensorPosY > max_y/0.01
            continue;
        end
        if Obs_Matrix(sensorPosY,sensorPosX) == 1
           k=k+1;
           % get the distance from centre to object
           distance(k) = sqrt((xdist)^2+(ydist)^2);
           objectDetected = 1;
        end
    end
end
if ~exist('objectDetected','var')
    objectDetected = 0;
end
if objectDetected == 1
    distance_min = min(distance);
end
end

