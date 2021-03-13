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
desired_vel = 0;
step = 0;
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
tenacity = 1;
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
scenario = 1;

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
    
    if abs(desired_psi - cur_psi)>0.1 
        step = step+1;
    elseif abs(desired_vel - cur_vel)>0.1
        step = step+1;
    else
        % when the desired heading and velocity have been achieved clear the PID
        % controller
        step=1;
        clear u_psi u_vel err_psi err_psi_d err_psi_i err_vel err_vel_d err_vel_i;
        err_psi_i(step) = 0;
        err_vel_i(step) = 0;
    end

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
%
%
% For left tenacity search anticlockwise (2), for right - clockwise (1)
if tenacity == 0
    search = 2;
else
    search = 1;
end
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
    [R] = calculateR(Rt,environment,'allowed',search,'T2');
    if R == Rt
       terminate=1;
       continue;
    end
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
   

    err_psi(step) = desired_psi_360 - cur_psi_360;
    % error shouldn't be bigger than 180 degrees:
    if abs(err_psi(step)) > pi
        err_psi(step)=-sign(err_psi(step))*(2*pi-abs(err_psi(step)));
    end
    % If robot has been told to stop or heading is 0, set vel to 0
    if stopRobot == 1 || abs(err_psi(step)) > 0.1
        desired_vel = 0;
    else
        [desired_vel] = getVelocity(goal,cur_x,cur_y);
    end
    % store desired velocity throughout simulation for later plotting
    desired_velocity(step) = desired_vel;
    
    err_vel(step) = desired_vel - cur_vel;
%---------------------------------------------------------------------%
        
    % PID Controllers for heading and velocity:

    Kp_psi = 20;  
    Ki_psi = 0.1;   
    Kd_psi = 0.1;  

    Kp_vel = 2.5;  %5
    Ki_vel = 35;   %70
    Kd_vel = .01;  %.01

    if step == 1
        prevStep = 1;
    else
        prevStep = step-1;
    end
    % For error in psi:
    %
    % Using Euler's backward rule
    err_psi_i(step) = err_psi_i(prevStep)  + err_psi(step)*dT; 
    err_psi_d(step) = (err_psi(step) - err_psi(prevStep))/dT;

    u_psi(step) = Kp_psi * err_psi(step) + Ki_psi * err_psi_i(step) +...
        + Kd_psi * err_psi_d(step) ;
    %
    %
    % For error in velocity:
    err_vel_i(step) = err_vel_i(prevStep)  + err_vel(step)*dT;
    err_vel_d(step) = (err_vel(step) - err_vel(prevStep))/dT;

    u_vel(step) = Kp_vel*err_vel(step)+Ki_vel*err_vel_i(step)+Kd_vel*err_vel_d(step);
%------------------------------------------------------------------%
% Convert inputs into voltages:    
    if abs(u_vel(step))> 12
        u_vel(step) = sign(u_vel(step))*12;
    end


    Vl = (u_vel(step) + u_psi(step))/2;
    Vr = (u_vel(step) - u_psi(step))/2;


    if Vl > 7.4 || Vl < -7.4
        Vl = sign(Vl)*7.4;
    end
    if Vr > 7.4 || Vr < -7.4
        Vr = sign(Vr)*7.4;
    end

    
     V_matrix(1,step) = Vl;
     V_matrix(2,step) = Vr;

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
    goalPlot(1) = plot(start(2),start(1),'Marker','x','MarkerEdgeColor','red',...
        'LineWidth',2,'MarkerSize',12);
    goalPlot(2) = plot(goal(2),goal(1),'Marker','x','MarkerEdgeColor','[0.9290, 0.6940, 0.1250]',...
        'LineWidth',2,'MarkerSize',12);
    set(goalPlot,'linestyle','none');
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
trajectory = plot(xio(:,20)+5,xio(:,19)+5,'k','LineStyle','--','LineWidth',2);
trajectory.Color(4) = 0.5;
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
legend([goalPlot(1:2) trajectory],{'Start','Goal','Path'},'FontSize',12); 

%----------------------------------------------%
toc;
%Plot Variables
% figure(2); plot(xio(:,20),xio(:,19));
% figure(3); plot(xio(:,19));
% figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
