function [R] = calculateR(Rt,layer,criteria,search,alg)

% This function find the closest clockwise/counterclockwise (based on
% tenacity) region that matches the input criteria (i.e. allowed or
% blocked).
if strcmp(alg,'EG')
 dummyEnv = layer.environment;
else
 dummyEnv = layer;
end
% find all allowed regions (including Rt)
Regions = find(strcmp(dummyEnv.sector,criteria));
% find the index of Rt sector in allowedRegions array
index = find(Regions==Rt);

% set conditions if Rt is the first or last allowed sector in the array
% (to simulate a circular array).
%
% Rright chooses next clockwise region, while Rleft chooses next ccw
% region

if index == 1
   Rright = Regions(end);
else
   Rright = Regions(index - 1);
end
if index == length(Regions)
    Rleft = Regions(1);
else
    Rleft = Regions(index + 1);
end

if search == 2
    R = Rleft;
elseif search == 1
    R = Rright;
end

end