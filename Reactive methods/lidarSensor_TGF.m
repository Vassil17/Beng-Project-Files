function [obstacleMap,scan,distance,objectDetected]=lidarSensor_TGF(obstacleMap,pose,range,cur_psi)
% 
% % Convert cur_psi which is in robot model notation to the standard notation 
% % (0 to 2pi ccw) starting at horizontal;
% if cur_psi < 0
%     cur_psi = cur_psi - 2*pi;
% end
% % Then add pi/2 as in robot model notation 0 angle is at vertical
% cur_psi = cur_psi + pi/2;
% %
pose = [pose 0];
% create a range sensor
rbsensor = rangeSensor;
[ranges,angles] = rbsensor(pose,obstacleMap);
scan = lidarScan(ranges,angles);
minRange = 0;
maxRange = range;

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
