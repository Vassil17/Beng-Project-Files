%function []=createSectorEnvironment_EG()
angles = scan.Angles;
for k=1:size(angles,1)
    if angles(k) < 0
        angles(k) = 2*pi + angles(k);
       
    end
end

% Split the circle around robot in K sectors and store them;
% the direction should be in global angle;
K=36;
previous_angle = -2*pi/K;
environment = struct;
blocked = 0;
show(obstacleMap); hold on;
for i=1:1:K
   environment.angle(i)= previous_angle + 2*pi/K;
   difference = angles - environment.angle(i);
   findObjects=find(difference>0 & difference<=0.1745);
   %if any(angles - environment.angle(i) > 0 && angles - environment.angle(i)<=0.1745)
   if isempty(findObjects)
       environment.sector(i) = "allowed";
       blocked = 0;
   else
       environment.sector(i) = "blocked";
       blocked = 1;
   end
   drawSectors(environment.angle(i),cur_x,cur_y,blocked)
   previous_angle = environment.angle(i);
   
end
drawnow;


%end