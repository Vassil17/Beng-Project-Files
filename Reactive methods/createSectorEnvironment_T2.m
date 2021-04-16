function [environment,Rt,obstacle_storage]=createSectorEnvironment_T2(scan,...
    K,angleToGoal,cur_x,cur_y,obstacle_storage,outer_loop)
%
%
% This function creates the angular sector distribution of the sensor
% radius of the robot used by the T2 algorithm. It also calculates where
% previous obstacles would be in the current angular sector distribution,
% storing them in short term memory.
%
%
%
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
       if mod(outer_loop,20)==0
       % Check if short term memory already contains this sector
           obstacle_storage.sector(end+1) =  i;
           obstacle_storage.angle(end+1) = environment.angle(i);
           obstacle_storage.distance(end+1) = environment.distance(i);
           obstacle_storage.centre(end+1,:) = [cur_y cur_x];
           blocked = 1;
%        end
       end % end mod
           
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


end