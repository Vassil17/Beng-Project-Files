clear;
clc;
% Create the obstacle map
max_x = 10;
max_y = 10;
resolution = 10;
obstacleMap = binaryOccupancyMap(max_x,max_y,resolution);

wall{1} = WallGeneration1(0,7.5,6,6,'h');
wall{2} = WallGeneration1(3,10,7,7,'h');
wall{3} = WallGeneration1(2,2,6,8,'v');
wall{4} = WallGeneration1(2,8,8,8,'h');
wall{5} = WallGeneration1(3,9,4,4,'h');
wall{6} = WallGeneration1(2,2,2,5,'v');
wall{7} = WallGeneration1(0,2,5,5,'h');
wall{8} = WallGeneration1(9,10,5,5,'h');
wall{9} = WallGeneration1(3,3,2,4,'v');
wall{10} = WallGeneration1(3,5,5,5,'h');
wall{11} = WallGeneration1(5,5,5,6,'v');
wall{12} = WallGeneration1(9,9,4,5,'v');
% Create range sensor
obstacleSensor = rangeSensor('HorizontalAngle', pi/2);
numReadings = obstacleSensor.NumReadings;
for counter=1:length(wall)
    clear x; clear y;
    for i=1:length(wall{counter})
        x(i) = wall{counter}(i,1);
        y(i) = wall{counter}(i,2);        
    end
    x=x.';
    y=y.';

    setOccupancy(obstacleMap, [x y], ones(i,1));
end
start_OG = [1 1];
goal_OG = [9 8];
% Convert location to grid (matrix) loaction:
start = [100 - 10*start_OG(1) 10*start_OG(2)];
goal = [100 - 10*goal_OG(1) 10*goal_OG(2)];
inflate(obstacleMap,0.2);
planner=plannerAStarGrid(obstacleMap);
path=plan(planner,start,goal);
startPoses = path(1:end-1,:);
endPoses = path(2:end,:);
rsConn = reedsSheppConnection;
rsPathSegs = connect(rsConn, startPoses, endPoses);
poses = [];
for i = 1:numel(rsPathSegs)
    lengths = 0:0.1:rsPathSegs{i}.Length;
    [pose, ~] = interpolate(rsPathSegs{i}, lengths);
    poses = [poses; pose];
end
show(obstacleMap);
grid on; hold on;
for i=1:20:size(poses,1)
   plot(poses(:,2),poses(:,1));
end