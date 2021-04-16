function [layer] = sensorDataFilter(layer,K,scan)
%index = find(strcmp(layer.environment.sector,'blocked'));
% get the angles of these sectors
%angles = layer.environment.angle(index);
memoryAngles = layer.obstacle_storage.angle;
layer.obstacle_storage.obstacle = [];


% Check each sector 
% for k=1:length(angles)
%     if (any(abs(angles(k) - memoryAngles)<0.2)) || isempty(memoryAngles)
%     % if continuous add to memory
%         layer.environment.sector(index(k)) = 'blocked';
%     else
%     % otherwise label as allowed
%         layer.environment.sector(index(k)) = 'allowed';
%     end
% end

% Find where there is a gap in the obstacle (discontinuity)
angles = scan.Angles;
index = find(abs(angles(1:(end-1))-angles(2:end))>0.2);
if ~isempty(index)
    for counter=1:length(angles)
        if angles(counter) < 0
            angles(counter) = 2*pi + angles(counter);
        end
    end
    diff = angles(index) - layer.environment.angle;
    sectors=find(diff>0 & diff<=2*pi/K);
    layer.environment.sector(sectors) = 'allowed';
end