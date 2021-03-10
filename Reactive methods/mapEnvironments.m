function [obstacleMap,start,goal]=mapEnvironments(resolution,scenario)
%
% This function takes an image file of an environment as an input and
% converts it into a binary occupancy map. Each scenario has a specific
% start and goal points that can be modified in this function.
%
%
%
% Scenario 1 is a cluttered room
% Scenario 2 is a simple maze
% Scenario 3 is a simple obstacle avoidance problem
if scenario == 1
    map = 'scenario1.png';
    start = [1 2];
    goal = [8 4];    
    %goal = [3 2];
elseif scenario == 2
    map = 'scenario2.png';
    start = [7 1];
    goal = [9 8];
elseif scenario == 3
    map = 'scenario3.png';
    start = [5.5 5];
    goal = [8 7];
elseif scenario == 4
    map = 'scenario4.png';
    start = [3.5 2];
    goal = [6 6];
elseif scenario == 5
    map = 'scenario5.png';
    start = [3 1];
    goal = [1 2];
elseif scenario == 6
    map = 'scenario6.png';
    start = [5 1];
    goal = [5 8];
elseif scenario == 7
    map = 'scenario7.png';
    start = [5 1];
    goal = [5 9];
end

    image = imread(map);
    grayimage = rgb2gray(image);
    bwimage = grayimage < 0.5;
    obstacleMap = binaryOccupancyMap(bwimage,1000/resolution);


end