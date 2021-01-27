function [obstacleMap,scan,distance,objectDetected]=lidarSensor_EG(obstacleMap,pose)
% for EG use global LIDAR angle (i.e. always at angle pi/2 which is 0 on
% this model)
sensorAngle = 0; %pi/2
pose = [pose sensorAngle];
% create a range sensor
rbsensor = rangeSensor;
[ranges,angles] = rbsensor(pose,obstacleMap);
scan = lidarScan(ranges,angles);
minRange = 0;
maxRange = 2;

scan = removeInvalidData(scan,'RangeLimits',[minRange maxRange]);
% rotate the scan to match robot model frame of reference
%scan = transformScan(scan,[0,0,sensorAngle-pi/2]);
if scan.Count ~= 0 
    objectDetected = 1;
    distanceLeft = sqrt(scan.Cartesian(1,1).^2 + scan.Cartesian(1,2).^2);
    distanceRight = sqrt(scan.Cartesian(end,1).^2 + scan.Cartesian(end,2).^2);
    distanceMin = min(sqrt(scan.Cartesian(:,1).^2 + scan.Cartesian(:,2).^2));
    distance = [distanceLeft distanceRight distanceMin];
else 
    objectDetected = 0;
    distance = [NaN NaN NaN];
end
end
