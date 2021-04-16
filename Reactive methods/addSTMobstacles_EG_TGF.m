function [layer] = addSTMobstacles_EG_TGF(layer,K,removefromSTM,Rt)
% This memory checks the short term memory and adds the obstacles to the
% layer
%
%
%
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
        layer.environment.sector(newSector(sector)) = 'blocked';
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

if exist('removefromSTM','var')
    for p=1:length(removefromSTM)
        sector = removefromSTM(p);
        layer.environment.sector(sector) = 'allowed';
    end
end

end