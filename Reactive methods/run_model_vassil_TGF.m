% Version 11 - Reactive Tangential Gap Flow
%
%
%
% 
%----------------------------------------------%
% Workspace Clear up
close all;
clear;
clc;
%----------------------------------------------%

%----------------------------------------------%
% Setup Simulation
goal = [9 2];
sim_time = 200;
dT = 0.05;
point = 1;
xi = zeros(1,24); % initial state for x
% define starting position (in original coordinates, i.e. subtract 5 from
% map coordinates)
xi(19) = 2;
xi(20) = -3;
LeftS = 0;
RightS = 0;
err_psi_i(1) = 0;
err_vel_i(1) = 0;
stopRobot = 0;
state = 1;
desired_psi = 0;
desired_psi_360=0;
originalPosition = 0;
check_for_goal = 0;
isPathValid = 0;
go_to_goal = 0;
% This section is to create the sector distribution of the LiDAR scan
K=24;
environment = struct;
% Initiate the obstacle_storage structure
obstacle_storage = struct;
obstacle_storage.sector = [];
obstacle_storage.angle = [];
obstacle_storage.distance = [];
obstacle_storage.centre = [];
% set tenacity (0 for left, 1 for right)
tenacity = 1;
%
%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;
resolution = 10;
% Create the obstacle map
obstacleMap = binaryOccupancyMap(max_x,max_y,resolution);


wall{1} = WallGeneration1(0,7,8,8,'h');
%wall{2} = WallGeneration1(3,10,8,8,'h');
wall{2} = WallGeneration1(0,0,6,8,'v');
wall{3} = WallGeneration1(7,7,6,8,'v');
% wall{4} = WallGeneration1(2,8,8,8,'h');
% wall{5} = WallGeneration1(3,9,4,4,'h');
% wall{6} = WallGeneration1(2,2,2,5,'v');
% wall{7} = WallGeneration1(0,2,5,5,'h');
% wall{8} = WallGeneration1(9,10,5,5,'h');
% wall{9} = WallGeneration1(3,3,2,4,'v');
% wall{10} = WallGeneration1(9,9,4,5,'v');
% wall{11} = WallGeneration1(0,0,5,6,'v');
% wall{12} = WallGeneration1(10,10,5,8,'v');

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

%-----------------------------------------------------------------------%
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
       
% Run LiDAR sensor
       pose = [cur_y cur_x];
       [obstacleMap,scan,distance,objectDetected]=lidarSensor_EG(obstacleMap,pose);
     


%-------------------------------------------------------------------------%
% Behavior
%
% Check direct path to goal
% Code below is not necessary for this version
%
%     path = [cur_y cur_x 0;[goal(2) goal(1)] 0];
%     validator = validatorOccupancyMap;
%     validator.Map = obstacleMap;
%     validator.ValidationDistance = 0.01;
%     if isStateValid(validator,path)
%         startStates = path(1,:);
%         endStates = path(2,:);
%         [isPathValid] = isMotionValid(validator,startStates,endStates);
%         if isPathValid == 1
%             go_to_goal=1;
%             [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,goal);
%             if at_waypoint == 1
%                 stopRobot=1;
%                 break;
%             end
%         else
%             go_to_goal = 0;
%         end
%     else
%         go_to_goal = 0;
%     end
%
%
%
% check the angle to the goal
[at_waypoint,angleToGoal] = los_auto(cur_x,cur_y,goal);
if at_waypoint == 1
    break;
end 
% run script to plot the sector distribution and obtain target path sector
% Rt
[environment,Rt,obstacle_storage] = createSectorEnvironment_EG(scan,K,angleToGoal,...
    cur_x,cur_y,obstacle_storage);

% if Rt sector is allowed choose it
if strcmp(environment.sector(Rt),'allowed')
    Rleft = Rt;
    Rright = Rt;
    % and erase the short term memory;
    obstacle_storage.sector = [];
    obstacle_storage.angle = [];
    obstacle_storage.distance = [];
    obstacle_storage.centre = [];
else
    % mark the Rt sector as allowed in order to search its neighbours
    environment.sector(Rt) = 'allowed';
    % find all allowed regions (including Rt)
    allowedRegions = find(strcmp(environment.sector,'allowed'));
    % find the index of Rt sector in allowedRegions array
    index = find(allowedRegions==Rt);
    % set conditions if Rt is the first or last allowed sector in the array
    % (to simulate a circular array).
    %
    % Rright chooses next clockwise region, while Rleft chooses next ccw
    % region
    if index == 1
       Rright = allowedRegions(end);
    else
       Rright = allowedRegions(index - 1);
    end
    if index == length(allowedRegions)
        Rleft = allowedRegions(1);
    else
        Rleft = allowedRegions(index + 1);
    end
    % set Rt sector back to blocked
    environment.sector(Rt) = 'blocked';
end
if tenacity == 0
desired_angle = environment.angle(Rleft);
else
    desired_angle = environment.angle(Rright);
end

% Convert back to the coordinate system of the model (-pi to 0 to pi)
if desired_angle >= pi
    desired_psi = 2*pi - desired_angle;
elseif desired_angle < pi
    desired_psi = -desired_angle;
end
% Add pi/2 to shift the 0 to be on the vertical axis
desired_psi = desired_psi + pi/2;
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
        [desired_vel] = getVelocity(goal,cur_x,cur_y);
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
    clf; show(obstacleMap);grid on; hold on;
    xlabel('X [metres]');
    ylabel('Y [metres]');
    drawrobot(0.2,xi(20)+5,xi(19)+5,xi(24),'b');
    % draw the sectors 

        for i=1:1:K
        drawSectors(environment.angle(i),cur_x,cur_y,environment.sector(i),K,i,Rright,Rleft,tenacity)
        end

    pause(0.001);
    %----------------------------------------------%
    
end
%----------------------------------------------%
% Plot which points the robot reached
figure(1);
for i=1:1:size(goal,1)
   % plot(robot_path(i,2),robot_path(i,1),'-x');
    plot(xio(:,20)+5,xio(:,19)+5,'k');
end
%----------------------------------------------%
toc;
%Plot Variables
% figure(2); plot(xio(:,20),xio(:,19));
% figure(3); plot(xio(:,19));
% figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
