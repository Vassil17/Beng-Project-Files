function [obstacleMap,start,goal]=mapEnvironments(resolution,scenario)
%
% This function draws the specific map for the different test scenarios
%
%
%
% Scenario 1 is a cluttered room
% Scenario 2 is a simple maze
if scenario == 1
    map = 'scenario1.png';
    start = [1 2];
    goal = [9.5 6];    
    %goal = [3 2];
elseif scenario == 2
    map = 'scenario2.png';
    start = [7 1];
    goal = [9 8];

end

    image = imread(map);
    grayimage = rgb2gray(image);
    bwimage = grayimage < 0.5;
    obstacleMap = binaryOccupancyMap(bwimage,1000/resolution);


end