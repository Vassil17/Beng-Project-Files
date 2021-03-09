function [layer,activeLayer,newLayer,...
    prevEnvironment,Rt,prevRt]=createSectorEnvironment_EG(layer,activeLayer,scan,...
    K,angleToGoal,cur_x,cur_y,outer_loop)

% This function creates the angular sector distribution of the sensor
% radius of the robot used by the EG algorithm. It also calculates where
% previous obstacles would be in the current angular sector distribution,
% storing them in short term memory.

% If all regions are blocked, then a new activeLayer is added and is made active,
% denoted by the variable activeLayer. Only the last one and the current activeLayer 
% are ever stored.
newLayer = 0;
% Check if sectors exist (needed for first run of programme)
% store the environment from last step as (1).
if isfield(layer(activeLayer),'environment') && ...
        isfield(layer(activeLayer).environment,'sector')
    prevEnvironment = layer(activeLayer).environment;
    prevRt = layer(activeLayer).environment.Rt;
    if all(strcmp(layer(activeLayer).environment.sector,'blocked'))
        % create a new activeLayer and make it active
        activeLayer = activeLayer+1;
        % remember at which step this occured
        newLayer = 1;
        % Need to re-initialise the obstacle_storage structure for new
        % layer
        layer(activeLayer).obstacle_storage = [];
        layer(activeLayer).obstacle_storage.sector = [];
        layer(activeLayer).obstacle_storage.angle = [];
        layer(activeLayer).obstacle_storage.distance = [];
        layer(activeLayer).obstacle_storage.centre = [];
    end
end
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
   if mod(outer_loop,50) == 0
try       layer(activeLayer).obstacle_storage.sector(end+1) =  i;
catch
    x
end
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

try cond = ~isempty(layer(activeLayer).obstacle_storage.sector);
catch
    x
end
if cond
    % Map previous banned regions to current layer(activeLayer).environment   
    for counter=1:length(layer(activeLayer).obstacle_storage.sector)
        d = layer(activeLayer).obstacle_storage.distance(counter);
        Angle = layer(activeLayer).obstacle_storage.angle(counter);
        tx = layer(activeLayer).environment.centre(1) - layer(activeLayer).obstacle_storage.centre(counter,1);
        ty = layer(activeLayer).environment.centre(2) - layer(activeLayer).obstacle_storage.centre(counter,2);
        transMatrix = [1 0 tx; 0 1 ty; 0 0 1]; 
       % define distance to obstacle and angle of sector as a vector
       vector = [d*cos(Angle); d*sin(Angle); 1];
       newVector = transMatrix\vector;
       newAngle = atan2(newVector(2),newVector(1));
       if newAngle < 0
         newAngle = newAngle + 2*pi;
       end
       % if new angle sector doesnt match the old angle sector then block
       % new sector
       diff = newAngle - layer(activeLayer).environment.angle;
       % The difference should be between 0 and the angle size of each
       % sector (small tolerance added to skew it towards the next sector);
       newSector = find(diff>=-0.01 & diff<2*pi/K-1e-5);
       for sector=1:length(newSector)
            layer(activeLayer).environment.sector(newSector(sector)) = 'blocked';
       end
       % This for loop makes sure that if a sector state is allowed but
       % the one after it and the one before it are blocked, then block the
       % middle one too.
       % This is necessary due to the transformations and the way the
       % sectors are designed.
%        for count = 1:length(layer(activeLayer).environment.sector)
%            % This if nest ensures that the adjacent sectors are checked
%            % cyclically (1-2-3...-K-1-2-3-4...)
%            if count == length(layer(activeLayer).environment.sector)-1 
%                first = count;
%                middle = count+1;
%                last = 1;
%            elseif count == length(layer(activeLayer).environment.sector)
%                first = count;
%                middle = 1;
%                last = 2;               
%            else
%                first = count;
%                middle = count+1;
%                last = count+2;     
%            end
%            if strcmp(layer(activeLayer).environment.sector(first),'blocked') &&...
%                    strcmp(layer(activeLayer).environment.sector(last),'blocked')
%               layer(activeLayer).environment.sector(middle) = 'blocked'; 
%            end
%        end
    end
end
layer(activeLayer).environment.Rt = Rt;
% If the prevEnvironment wasn't assigned (i.e. at first loop or when the
% layer has been cleaned), assign it
if ~exist('prevEnvironment','var')
   prevEnvironment = layer(activeLayer).environment;
   prevRt = layer(activeLayer).environment.Rt;
end
end