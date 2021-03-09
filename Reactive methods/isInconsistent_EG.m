function [cond]=isInconsistent_EG(Rcheck,layer,sensorData)
% This function checks whether a region's state stored in memory is 
% consistent with the current information from the sensor.
%
%
%
try cond = ~strcmp(layer.environment.sector(Rcheck),...
    sensorData.environment.sector(Rcheck));
catch
    x
end


end