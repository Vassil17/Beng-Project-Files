function [environment,obstacle_storage] = addSTMobstacles_T2(environment,obstacle_storage,K,Rt)
% This memory checks the short term memory and adds the obstacles to the
% layer
%
%
%
% Map previous banned regions to current layer(activeLayer).environment   
for counter=1:length(obstacle_storage.sector)
    d = obstacle_storage.distance(counter);
    Angle = obstacle_storage.angle(counter);
    tx = environment.centre(1) - obstacle_storage.centre(counter,1);
    ty = environment.centre(2) - obstacle_storage.centre(counter,2);
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
   diff = newAngle - environment.angle;
   % The difference should be between 0 and the angle size of each
   % sector (small tolerance added to skew it towards the next sector);
   newSector = find(diff>=-0.01 & diff<2*pi/K-1e-5);
   for sector=1:length(newSector)
        environment.sector(newSector(sector)) = 'blocked';
   end

end

end