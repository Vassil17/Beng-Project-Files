function [layer,sensorData,activeLayer,...
    prevEnvironment,Rt,prevRt]=createSectorEnvironment_EG_v2(layer,activeLayer,scan,...
    K,angleToGoal,cur_x,cur_y,outer_loop)

% This function creates the angular sector distribution of the sensor
% radius of the robot used by the EG algorithm. It also calculates where
% previous obstacles would be in the current angular sector distribution,
% storing them in short term memory.


% Split the circle around robot in K sectors and store them;
% the direction should be in global angle;
% start at negative angle so that the angle of the sector indicates its
% ending position (i.e. first sector is from 0 to 0.1745 radians for 36 sectors)
previous_angle = -2*pi/K;
blocked = 0;
angles = scan.Angles;
% the negative angles are clockwise -> change them so that all the Lidar
% scan effectively sweeps from 0(which is 90 in global coordinates) to 360 counterclockwise.
for k=1:size(angles,1)
    if angles(k) < 0
        angles(k) = 2*pi + angles(k);
       
    end
end

if angleToGoal <= 0
    angleToGoal = -angleToGoal;
elseif angleToGoal > 0
   angleToGoal = 2*pi - angleToGoal; 
end
angleToGoal =  angleToGoal + pi/2;
if angleToGoal >= 2*pi
    angleToGoal = angleToGoal - 2*pi;
end
% Store the old layer as prevEnvironment
if isfield(layer(activeLayer),'environment') && ...
        isfield(layer(activeLayer).environment,'sector')
    prevEnvironment = layer(activeLayer).environment;
    prevRt = layer(activeLayer).environment.Rt;
end
%
% Clear the old layer environment and create a new one
clear layer(activeLayer).environment
for i=1:1:K
   layer(activeLayer).environment.angle(i)= previous_angle + 2*pi/K;
   difference = angles - layer(activeLayer).environment.angle(i);
   findObjects=find(difference>0 & difference<=2*pi/K);
   if isempty(findObjects)
       layer(activeLayer).environment.distance(i) = NaN;
       layer(activeLayer).environment.sector(i) = "allowed";
       blocked = 0;
   else
       % save the distance to the obstacle
       layer(activeLayer).environment.distance(i) = min(scan.Ranges(findObjects));
       layer(activeLayer).environment.sector(i) = "blocked";
   % Store the obstacles every X steps of the simulation
   if mod(outer_loop,20) == 0
       layer(activeLayer).obstacle_storage.sector(end+1) =  i;
       layer(activeLayer).obstacle_storage.angle(end+1) = layer(activeLayer).environment.angle(i);
       layer(activeLayer).obstacle_storage.distance(end+1) = layer(activeLayer).environment.distance(i);
       layer(activeLayer).obstacle_storage.centre(end+1,:) = [cur_y cur_x];
       blocked = 1;
   end      
   end
   if angleToGoal - layer(activeLayer).environment.angle(i) >=0 && angleToGoal - layer(activeLayer).environment.angle(i) <=2*pi/K
      Rt = i;
   end
   previous_angle = layer(activeLayer).environment.angle(i);
   
end
layer(activeLayer).environment.centre = [cur_y cur_x];
if ~exist('Rt','var')
   Rt=NaN;
end
% %%%%%%
% removeData = [];
% if exist('removefromSTM','var') && ~isempty(removefromSTM)
%     for p=1:length(removefromSTM)
%         sector = removefromSTM(p);
%         if strcmp(layer(activeLayer).environment.sector(sector),'blocked')
%             removeData(end+1) = p;
%         end
%     end
% end
% if ~isempty(removeData)
%     removefromSTM(removeData) = [];
% end
layer(activeLayer).environment.Rt = Rt;
% If the prevEnvironment wasn't assigned (i.e. at first loop or when the
% layer has been cleaned), assign it
if ~exist('prevEnvironment','var')
   prevEnvironment = layer(activeLayer).environment;
   prevRt = layer(activeLayer).environment.Rt;
end
% Save the data from the sensors (before STM is included) as sensorData
sensorData(activeLayer) = layer(activeLayer);
end