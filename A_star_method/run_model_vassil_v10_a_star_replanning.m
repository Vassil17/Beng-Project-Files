% Version 7 - LIDAR
%
%
%
% 
%----------------------------------------------%
% Workspace Clear up
close all;
clear all;
clc;
%----------------------------------------------%

%----------------------------------------------%
% Setup Simulation
goal = [9 8];
% goal and starting point for path need to be defined with angle due to the
% way MATLAB defines the planner
goal_path = [goal 0];
start = [6 1];
start_path = [start pi/2];
sim_time = 60;
dT = 0.05;
point = 1;
xi = zeros(1,24); % initial state for x
xi(19) = -4;
xi(20) = 1;
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
%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;
resolution = 10;

%
% Create the obstacle map
obstacleMap = binaryOccupancyMap(max_x,max_y,resolution);
estimatedMap = binaryOccupancyMap(max_x,max_y,resolution);

wall{1} = WallGeneration1(0,7,6,6,'h');
wall{2} = WallGeneration1(3,10,7,7,'h');
wall{3} = WallGeneration1(2,2,6,8,'v');
wall{4} = WallGeneration1(2,8,8,8,'h');
wall{5} = WallGeneration1(3,9,4,4,'h');
wall{6} = WallGeneration1(2,2,2,5,'v');
wall{7} = WallGeneration1(0,2,5,5,'h');
wall{8} = WallGeneration1(8,10,5,5,'h');
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
    setOccupancy(estimatedMap, [x y], ones(i,1));
end
object{1} = WallGeneration1(6,6,4,5,'v');
object{2} = WallGeneration1(7,7,5,6,'v');
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
%----------------------------------------------%
tic;
%----------------------------------------------%
for outer_loop = 1:(sim_time/dT)
    %----------------------------------------------%
    %
    %
    %
    % CONTROLLER SECTION
    cur_x = xi(19)+5;
    cur_y = xi(20)+5;
    cur_psi = xi(24);
    cur_vel = xi(13);
    n = outer_loop;

    % Change heading to be from 0 to 360 degrees   

    if cur_psi < 0 
        cur_psi_360 = cur_psi + 2*pi;
    elseif cur_psi >= 2*pi
        cur_psi_360 = cur_psi - 2*pi;
    else  
        cur_psi_360 = cur_psi;
    end
       

     % Set up sensor position (x,y) for each sensor for psi=0;
    sensors = [cur_y cur_x; cur_y cur_x;];
    % Set up angle for each sensor
    sensorAngle = [cur_psi; cur_psi - pi/2];
      
    
%     % Create each sensor:
%     for i=1:size(sensors,1)
%         % rotation matrix isnt needed for now as sensors are at robot
%         % centre
% %       % use rotation matrix to find sensor position based on heading
% %       sensors(i,:) = transpose(([cos(cur_psi), -sin(cur_psi);sin(cur_psi), cos(cur_psi)]*sensors(i,:)'));
%        pose = [sensors(i,:)];
%        
%        [obstacleMap,scan(i),distance(i,:),objectDetected(i)]=lidarSensor(obstacleMap,pose,sensorAngle(i));
%     end

    
    

%-------------------------------------------------------------------------%
% Behavior - A* pathplanning
%
% Initially, create the plan:
    
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
        poses_inverted = poses;
        poses=[poses(:,2),poses(:,1)];
        current_point = current_point + 1;
    else
        % check sensor reading and
        % insert sensor reading into the estimatedMap
        [ranges, angles] = obstacleSensor([cur_y cur_x cur_psi], obstacleMap);
        insertRay(estimatedMap, [cur_y cur_x cur_psi], ranges, angles, ...
        obstacleSensor.Range(end));
        %  drawnow;
        if any(checkOccupancy(estimatedMap,poses_inverted(:,1:2)))
            current_point = 1;
            validator.Map = estimatedMap;
            planner = plannerHybridAStar(validator,'MinTurningRadius',0.64);
            % Create plan
            path = plan(planner,[cur_y cur_x cur_psi],goal_path);
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
            poses_inverted = poses;
            poses=[poses(:,2),poses(:,1)];
        else              
        % Tell robot to follow each point
        % goal_coordinate flips x and y coordinates (otherwise it doesnt
        % work)
        goal_coordinate = [endPoses(current_point,2),endPoses(current_point,1)];
        [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,goal_coordinate);  
        if all(abs([cur_y cur_x] - goal_path(1,1:2)) < 0.2)
            break;
        end
        % Once robot has reached a point, move to the next
        if all(abs([cur_x cur_y] - goal_coordinate) < 0.2)
           if current_point == size(endPoses,1)
                break;
           else
                current_point = current_point + 1;
            end
        end
        end
    end

%
%
%-------------------------------------------------------------------------%    

    %---------------------------------------------------------------------%
    % Once we've obtained the desired heading a controller needs to be
    % designed to feed appropriate voltages into the motors
    %
    % First calculate error err_psi between current and desired heading,
    % then error between current and desired distance
        
% 
    if desired_psi < 0
    desired_psi_360 = desired_psi + 2*pi;
    
    elseif desired_psi >= 2*pi
        desired_psi_360 = desired_psi - 2*pi;
    else
        desired_psi_360 = desired_psi;
    end
   
    
    err_psi(n) = desired_psi_360 - cur_psi_360;
    % error shouldn't be bigger than 180 degrees:
    if abs(err_psi(n)) > pi
        err_psi(n)=-sign(err_psi(n))*(2*pi-abs(err_psi(n)));
    end
    % If robot has been told to stop or heading is 0, set vel to 0
    if stopRobot == 1 || abs(err_psi(n)) > 0.1
        desired_vel = 0;
    else
        [desired_vel] = getVelocity(goal(point,:),cur_x,cur_y);
    end
    % store desired velocity throughout simulation for later plotting
    desired_velocity(n) = desired_vel;
    
    err_vel(n) = desired_vel - cur_vel;
%---------------------------------------------------------------------%
        
    % PID Controllers for heading and velocity:

    Kp_psi = 20;  
    Ki_psi = 0.1;   
    Kd_psi = 0.1;  

    Kp_vel = 2.5;  %5
    Ki_vel = 35;   %70
    Kd_vel = .01;  %.01

    if n == 1
        prev_n = 1;
    else
        prev_n = n-1;
    end
    % For error in psi:
    %
    % Using Euler's backward rule
    err_psi_i(n) = err_psi_i(prev_n)  + err_psi(n)*dT; 
    err_psi_d(n) = (err_psi(n) - err_psi(prev_n))/dT;

    u_psi(n) = Kp_psi * err_psi(n) + Ki_psi * err_psi_i(n) +...
        + Kd_psi * err_psi_d(n) ;
    %
    %
    % For error in velocity:
    err_vel_i(n) = err_vel_i(prev_n)  + err_vel(n)*dT;
    err_vel_d(n) = (err_vel(n) - err_vel(prev_n))/dT;

    u_vel(n) = Kp_vel*err_vel(n)+Ki_vel*err_vel_i(n)+Kd_vel*err_vel_d(n);
%------------------------------------------------------------------%
% Convert inputs into voltages:    
    if abs(u_vel(n))> 12
        u_vel(n) = sign(u_vel(n))*12;
    end


    Vl = (u_vel(n) + u_psi(n))/2;
    Vr = (u_vel(n) - u_psi(n))/2;


    if Vl > 7.4 || Vl < -7.4
        Vl = sign(Vl)*7.4;
    end
    if Vr > 7.4 || Vr < -7.4
        Vr = sign(Vr)*7.4;
    end

    
     V_matrix(1,n) = Vl;
     V_matrix(2,n) = Vr;

    %
    %
    %
    %
    %
    %----------------------------------------------%
    % Run Model
    Va = [Vl; Vl; Vr; Vr];
    [xdot, xi] = full_mdl_motors(Va,xi,0,0,0,0,dT);   
    xi = xi + (xdot*dT); % Euler intergration
    
    % Store varibles
    xdo(outer_loop,:) = xdot;
    xio(outer_loop,:) = xi;
    %----------------------------------------------%
    
    %----------------------------------------------%
    figure(1);
    clf; show(obstacleMap);
    grid on; hold on;
    drawrobot(0.2,xi(20)+5,xi(19)+5,xi(24),'b');
    plot(goal(1),goal(2),'-o');
    for i=1:size(poses,1)
       plot(poses(:,2),poses(:,1),'-x');
    end
%     for i=1:length(sensors)
%         drawSensorCone(sensorAngle(i),xi(19)+sensors(i,1),xi(20)+sensors(i,2),1);
%     end
%    xlabel('x, m'); ylabel('y, m');  
%     for counter=1:length(wall)
%         plot(wall{counter}(:,1),wall{counter}(:,2),'k-');   
%     end
    pause(0.001);
    %----------------------------------------------%
    
end
toc;
%----------------------------------------------%
% Plot which points the robot reached
figure(1);
for i=1:1:size(goal,1)
   % plot(robot_path(i,2),robot_path(i,1),'-x');
    plot(xio(:,20)+5,xio(:,19)+5,'k');
end
%----------------------------------------------%
%Plot Variables
% figure(2); plot(xio(:,20),xio(:,19));
% figure(3); plot(xio(:,19));
% figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
