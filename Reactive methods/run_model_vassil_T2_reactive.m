% Version 11 - Reactive T2 / EG algorithm
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
sim_time = 500;
dT = 0.05;
point = 1;
xi = zeros(1,24); % initial state for x
% define starting position (in original coordinates, i.e. subtract 5 from
% map coordinates)
xi(19) = 2;
xi(20) = -4;
LeftS = 0;
RightS = 0;
err_psi_i(1) = 0;
err_vel_i(1) = 0;
stopRobot = 0;
desired_psi = 0;
desired_psi_360=0;
%
% LiDAR Range
range = 1;
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
tenacity = 0;
% set terminate to 0
terminate = 0;
%
%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;
resolution = 10;
% Choose scenario (start and goal defined in scenarios)
scenario = 5;

[obstacleMap,start,goal]=mapEnvironments(resolution,scenario);
xi(19) = start(1) - 5;
xi(20) = start(2) - 5;

% Store the current location and heading for later plotting
plotStorage(1,:) = [start(2) start(1) xi(24)];
sectorPlotStorage = [];
% Dummy variable to count sectorPlotStorage
qq = 0;
%-----------------------------------------------------------------------%
tic;
%----------------------------------------------%

for outer_loop = 1:(sim_time/dT)
    if terminate == 1
        break;
    end
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
       [obstacleMap,scan,distance,objectDetected]=lidarSensor_T2(obstacleMap,pose,range);
     


%-------------------------------------------------------------------------%
% Navigation Section
%
%
% check the angle to the goal
[at_waypoint,angleToGoal] = los_auto(cur_x,cur_y,goal);
if at_waypoint == 1
    break;
end 
% run script to plot the sector distribution and obtain target path sector
% Rt
[environment,Rt,obstacle_storage] = createSectorEnvironment_T2(scan,K,angleToGoal,...
    cur_x,cur_y,obstacle_storage,outer_loop);

% if Rt sector is allowed choose it
if strcmp(environment.sector(Rt),'allowed')
    R = Rt;
    % and erase the short term memory;
    obstacle_storage.sector = [];
    obstacle_storage.angle = [];
    obstacle_storage.distance = [];
    obstacle_storage.centre = [];
else
    % mark the Rt sector as allowed in order to search its neighbours
    environment.sector(Rt) = 'allowed';
    [R] = calculateR(Rt,environment,'allowed',tenacity,'T2');
    environment.sector(Rt) = 'blocked';
end

desired_angle = environment.angle(R);

% Convert back to the coordinate system of the model (-pi to 0 to pi)
if desired_angle >= pi
    desired_psi = 2*pi - desired_angle;
elseif desired_angle < pi
    desired_psi = -desired_angle;
end
% Add pi/2 to shift the 0 to be on the vertical axis
desired_psi = desired_psi + pi/2;
%-------------------------------------------------------------------------%    
% Control Section
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
    % Plot the robot and sectors
    %----------------------------------------------%
    % refresh plot every X steps
    if mod(outer_loop,5)==0
    figure(1);
    clf; show(obstacleMap);grid on; hold on;
    xlabel('X position [m]');
    ylabel('Y position [m]');
    title('Map of the environment');
    drawrobot(0.2,xi(20)+5,xi(19)+5,xi(24),'b');
    % plot the goal point
    goal_marker = plot(goal(2),goal(1),'Marker','x','MarkerFaceColor','black',...
        'LineWidth',2,'MarkerSize',12);

    % draw the sectors 
    for i=1:1:K
        drawSectors(environment.angle(i),cur_x,cur_y,environment.sector(i),K,i,R,range)
    end
    end
    if qq ==0
        qq=qq+1;
        sectorPlotStorage(qq).env = environment;
        sectorPlotStorage(qq).pos = [cur_x cur_y];
        sectorPlotStorage(qq).R = R;
        condition = 1;
    else
        condition = mod(outer_loop,100);
    end
    if condition == 0
        qq = qq+1;
        plotStorage(end+1,:) =  [xi(20)+5 xi(19)+5 xi(24)];
        sectorPlotStorage(qq).env = environment;
        sectorPlotStorage(qq).pos = [cur_x cur_y];
        sectorPlotStorage(qq).R = R;
    end
    pause(0.001);
 %----------------------------------------------%
    
end

%----------------------------------------------%
% Plot which points the robot reached
figure(1);
trajectory = plot(xio(:,20)+5,xio(:,19)+5,'k','LineWidth',2);
% for q = 1:size(plotStorage,1)
% drawrobot(0.2,plotStorage(q,1),plotStorage(q,2),plotStorage(q,3),'b');
% end
% for cntr = 1:size(sectorPlotStorage,2)
%     for k=1:K
%         angle = sectorPlotStorage(cntr).env.angle(k);
%         sector = sectorPlotStorage(cntr).env.sector(k);
%         cur_x = sectorPlotStorage(cntr).pos(1);
%         cur_y = sectorPlotStorage(cntr).pos(2);
%         R = sectorPlotStorage(cntr).R;
%         drawSectors(angle,cur_x,cur_y,sector,K,k,R,range)
%     end
% end
legend([goal_marker,trajectory],{'Target','Path'});

%----------------------------------------------%
toc;
%Plot Variables
% figure(2); plot(xio(:,20),xio(:,19));
% figure(3); plot(xio(:,19));
% figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
