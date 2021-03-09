function [layer] = addSTMobstacles_EG(layer,K,removefromSTM,Rt)
% This memory checks the short term memory and adds the obstacles to the
% layer
%
%
%
% Create an array for removing data from the short term memory
removeSTMdata = [];
% Map previous banned regions to current layer(activeLayer).environment   
for counter=1:length(layer.obstacle_storage.sector)
    d = layer.obstacle_storage.distance(counter);
    Angle = layer.obstacle_storage.angle(counter);
    tx = layer.environment.centre(1) - layer.obstacle_storage.centre(counter,1);
    ty = layer.environment.centre(2) - layer.obstacle_storage.centre(counter,2);
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
   diff = newAngle - layer.environment.angle;
   % The difference should be between 0 and the angle size of each
   % sector (small tolerance added to skew it towards the next sector);
   newSector = find(diff>=-0.01 & diff<2*pi/K-1e-5);
   for sector=1:length(newSector)
       if any(newSector(sector) == removefromSTM)
           removeSTMdata(end+1) = counter;
           removeSector = newSector(sector);
           layer.environment.sector(newSector(sector)) = 'allowed';
       else      
           layer.environment.sector(newSector(sector)) = 'blocked';
       end
   end
   
end

if ~isempty(removeSTMdata)
    removefromSTM(removefromSTM == removeSector) = [];
    layer.obstacle_storage.sector(removeSTMdata) = [];
    layer.obstacle_storage.angle(removeSTMdata) = [];
    layer.obstacle_storage.distance(removeSTMdata) = [];
    layer.obstacle_storage.centre(removeSTMdata,:) = [];
end

end