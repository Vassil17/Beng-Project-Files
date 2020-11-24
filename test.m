clear;
clc;
% Create Environment
max_x = 10;
max_y = 10;
resolution = 10;
% Create the obstacle map
obstacleMap = binaryOccupancyMap(max_x,max_y,resolution);



%Obs_Matrix = zeros(max_x/0.01,max_y/0.01);
%visionMatrix = zeros(max_x/0.01,max_y/0.01);
% walls are now contained in a cell
wall{1} = WallGeneration1(6,6,4,6,'v');
%wall{1} = WallGeneration1(-1, 1,-2,-2,'h');
wall{2} = WallGeneration1(2, 5, 3, 3,'h');
% wall{3} = WallGeneration1(-1, 1,0.5,0.5,'h');
% wall{4} = WallGeneration1(-1, -1, -0.5, 0.49,'v');
% wall{5} = WallGeneration1(-3, -1,-0.49,-0.49,'h');
% % wall{6}

for counter=1:length(wall)
    for i=1:length(wall{counter})
        x(i) = wall{counter}(i,1);
        y(i) = wall{counter}(i,2);        
    end
    if counter == 1
        x=x.';
        y=y.';
    else 
        x=x;
        y=y;
    end
    setOccupancy(obstacleMap, [x y], ones(i,1)) 
    
end
figure;
pose = [5 5 0];
rbsensor = rangeSensor;
[ranges,angles] = rbsensor(pose,obstacleMap);
maxrange = 5;
scan = lidarScan(ranges,angles);
insertRay(obstacleMap,pose,scan,maxrange);
show(obstacleMap);