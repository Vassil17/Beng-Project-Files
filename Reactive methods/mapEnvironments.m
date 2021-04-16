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
    map = 'scenario1_new.png';
   start = [1 4]; 
  % start = [1 3];
    goal = [8 4];   %4 
elseif scenario == 2
    map = 'scenario1_estimated.png';
    start = [1 4];
    goal = [8 4]; 
%     map = 'scenario2.png';
%     start = [5.5 1];
%     goal = [8 7.8];
elseif scenario == 3
    map = 'scenario3.png';
    start = [3 9];
    goal = [6 0.5];
elseif scenario == 4
    map = 'scenario4.png';
    start = [3 9];
    goal = [6 0.5];
elseif scenario == 5
    map = 'scenario5.png';
    start = [3 1];
    goal = [1 5];
elseif scenario == 6
    map = 'scenario6.png';
    start = [5 1];
    goal = [5 8];
elseif scenario == 7
    map = 'scenario7.png';
    start = [5 1];
    goal = [5 9];
    %goal = [1 5];
elseif scenario == 8
    map = 'scenario8.png';
    start = [7 1];
    goal = [7 9];
elseif scenario == 9
    map = 'floor_plan_2_2.png';
    %start = [4 1];
    start = [4 9];
   % goal = [13.5 12];
    goal = [15 16];

elseif scenario == 10
    map = 'floor_plan_1.png';
    start = [4 1];
    goal = [10 14];
elseif scenario == 11
    map = 'scenario11.png';
    start = [6 5];
    goal = [3 5];
elseif scenario == 12
    map = 'scenario6_estimated.png';
    start = [5 1];
    goal = [5 8];
elseif scenario == 13
    map = 'scenario8_estimated.png';
    start = [7 1];
    goal = [7 9];
elseif scenario == 14
    map = 'floor_plan_1_estimated.png';
    start = [4 1];
    goal = [10 14];
elseif scenario == 15
    map = 'floor_plan_2_2_estimated.png';
    %start = [4 1];
    start = [4 9];
   % goal = [13.5 12];
    goal = [15 16];
elseif scenario == 16
    map = 'maze.jpg';
    start = [6.5 1];
    goal = [9 8];
end

    image = imread(map);
    grayimage = rgb2gray(image);
    bwimage = grayimage < 200;
    obstacleMap = binaryOccupancyMap(bwimage,100);


end