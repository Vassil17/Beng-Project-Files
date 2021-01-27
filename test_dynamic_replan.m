clear;
clc;
% Setup Simulation
goal = [9 7.5];
% goal and starting point for path need to be defined with angle due to the
% way MATLAB defines the planner
goal_path = [goal 0];
start = [5 5];
start_path = [start pi/2];
sim_time = 100;
dT = 0.05;
point = 1;
xi = zeros(1,24); % initial state for x
LeftS = 0;
RightS = 0;
err_psi_i(1) = 0;
err_vel_i(1) = 0;
stopRobot = 0;
state = 0;
desired_psi = 0;
desired_psi_360=0;
originalPosition = 0;
stateChanged = 0;
current_point = 0;
max_x = 10;
max_y = 10;
resolution = 10;
% Create the obstacle map
obstacleMap = binaryOccupancyMap(max_x,max_y,resolution);
estimatedMap = binaryOccupancyMap(max_x,max_y,resolution);
wall{1} = WallGeneration1(0,6,6,6,'h');
wall{2} = WallGeneration1(3,10,7,7,'h');
wall{3} = WallGeneration1(2,2,6,9,'v');
wall{4} = WallGeneration1(2,8,9,9,'h');
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
    setOccupancy(estimatedMap, [x y], ones(i,1));
end
object{1} = WallGeneration1(6,7.5,6,6,'h');
object{2} = WallGeneration1(6,6,7,8,'v');
for counter=1:length(object)
    clear x; clear y;
    for i=1:length(object{counter})
        x(i) = object{counter}(i,1);
        y(i) = object{counter}(i,2);        
    end
    x=x.';
    y=y.';

    setOccupancy(obstacleMap, [x y], ones(i,1)) 
    
end
poses=[1 1 1];
for counter=1:length(wall)
    clear x; clear y;
    for i=1:length(wall{counter})
        x(i) = wall{counter}(i,1);
        y(i) = wall{counter}(i,2);        
    end
    x=x.';
    y=y.';

    setOccupancy(obstacleMap, [x y], ones(i,1)) 

end


while current_point<=size(poses,1)
         
    
    if current_point == 0
        
        % Define algorithm to use for pathfinding
        % Create validator
        validator = validatorOccupancyMap;
        validator.Map = estimatedMap;
        planner = plannerHybridAStar(validator,'MinTurningRadius',0.64);
        % Create plan based on global map
        path = plan(planner,start_path,goal_path);
        % Create points for robot to follow
        path=path.States;
        startPoses = path(1:end-1,:);
        endPoses = path(2:end,:);
        rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
        rsPathSegs = connect(rsConn, startPoses, endPoses);
        poses = [];
        for i = 1:numel(rsPathSegs)
            lengths = 0:0.1:rsPathSegs{i}.Length;
            [pose, ~] = interpolate(rsPathSegs{i}, lengths);
            poses = [poses; pose];
        end
        %poses_inverted = poses;
        %poses=[poses(:,2),poses(:,1)];
        current_point = current_point + 1;
    else
        % check sensor reading and
        % insert sensor reading into the estimatedMap
        [ranges, angles] = obstacleSensor(poses(current_point,:), obstacleMap);
        insertRay(estimatedMap, poses(current_point,:), ranges, angles, ...
        obstacleSensor.Range(end));
        drawnow;
        if any(checkOccupancy(estimatedMap,poses(:,1:2)))
            current_point = 1;
            validator.Map = estimatedMap;
            planner = plannerHybridAStar(validator,'MinTurningRadius',0.64);
            % Create plan
            path = plan(planner,poses(current_point,:),goal_path);
            % Create points for robot to follow
            path=path.States;
            startPoses = path(1:end-1,:);
            endPoses = path(2:end,:);
            rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
            rsPathSegs = connect(rsConn, startPoses, endPoses);
            poses = [];
            for i = 1:numel(rsPathSegs)
                lengths = 0:0.1:rsPathSegs{i}.Length;
                [pose, ~] = interpolate(rsPathSegs{i}, lengths);
                poses = [poses; pose];
            end
        else              
            current_point = current_point+1;
        end
    end
end