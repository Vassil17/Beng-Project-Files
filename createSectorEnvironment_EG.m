function [environment,Rt,obstacle_storage]=createSectorEnvironment_EG(scan,...
    K,angleToGoal,cur_x,cur_y,obstacle_storage)
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
   environment.angle(i)= previous_angle + 2*pi/K;
   difference = angles - environment.angle(i);
   findObjects=find(difference>0 & difference<=2*pi/K);
   if isempty(findObjects)
       environment.distance(i) = NaN;
       environment.sector(i) = "allowed";
       blocked = 0;
   else
       % save the distance to the obstacle
       environment.distance(i) = min(scan.Ranges(findObjects));
       environment.sector(i) = "blocked";
       % Check if short term memory already contains this sector
       index = find(obstacle_storage.sector == i);
       % If it exists, rewrite it
       if ~isempty(index)
           obstacle_storage.sector(index) =  i;
           obstacle_storage.angle(index) = environment.angle(i);
           obstacle_storage.distance(index) = environment.distance(i);
           obstacle_storage.centre(index,:) = [cur_y cur_x];
           blocked = 1;
       else
           obstacle_storage.sector(end+1) =  i;
           obstacle_storage.angle(end+1) = environment.angle(i);
           obstacle_storage.distance(end+1) = environment.distance(i);
           obstacle_storage.centre(end+1,:) = [cur_y cur_x];
           blocked = 1;
       end
           
   end
   if angleToGoal - environment.angle(i) >=0 && angleToGoal - environment.angle(i) <=2*pi/K
      Rt = i;
   end
   previous_angle = environment.angle(i);
   
end
environment.centre = [cur_y cur_x];
if ~exist('Rt','var')
   Rt=NaN;
end

if ~isempty(obstacle_storage.sector)
    % Map previous banned regions to current environment   
    for counter=2:length(obstacle_storage.sector)
        d = obstacle_storage.distance(counter);
        Angle = obstacle_storage.angle(counter);
        tx = environment.centre(1) - obstacle_storage.centre(counter,1);
        ty = environment.centre(2) - obstacle_storage.centre(counter,2);
        transMatrix = [1 0 tx; 0 1 ty; 0 0 1]; 
       % define distance to obstacle and angle of sector as a vector
       vector = [d*cos(Angle); d*sin(Angle); 1];
       newVector = transMatrix\vector;
       newAngle = atan2(newVector(2),newVector(1));
       % if new angle sector doesnt match the old angle sector then block
       % new sector
       diff = newAngle - environment.angle;
       % The difference should be between 0 and the angle size of each
       % sector (small tolerance added to skew it towards the next sector);
       newSector = find(diff>=-1e-10 & diff<2*pi/K-1e-10);
       environment.sector(newSector) = 'blocked';

    end
end

end