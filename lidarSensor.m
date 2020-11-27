function [obstacleMap,scan,distance,objectDetected]=lidarSensor(obstacleMap,pose,sensorAngle)
% angles used by the lidarScan are +ccw starting from x axis;
% first shift by pi/2 (as 0 angle in this model is at pi/2 in MATLAB
% notation):
sensorAngle = pi/2 + sensorAngle;
% then change to ccw positive because model used cw positive:
sensorAngle = pi - sensorAngle;
pose = [pose sensorAngle];
% create a range sensor
rbsensor = rangeSensor;
[ranges,angles] = rbsensor(pose,obstacleMap);
scan = lidarScan(ranges,angles);
minRange = 0;
maxRange = 1;
% min and max angles are with relation to the pose angle (not global angle)
minAngle = -pi/12;
maxAngle = pi/12;

scan = removeInvalidData(scan,'RangeLimits',[minRange maxRange],...
    'AngleLimits',[minAngle maxAngle]);
% rotate the scan to match robot model frame of reference
scan = transformScan(scan,[0,0,sensorAngle-pi/2]);
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
