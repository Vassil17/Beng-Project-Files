% Version 11 - Reactive EG algorithm
% 
% This programme simulates a robot navigating its environment using a
% reactive navigation algorithm, called Escape Gap (EG). Different
% scenarios and environments can be tested and they are loaded up from the
% mapEnvironments.m function. 
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
goal = [9 6];
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
state = 1;
desired_psi = 0;
desired_psi_360=0;
originalPosition = 0;
check_for_goal = 0;
isPathValid = 0;
go_to_goal = 0;
desired_vel = 0;
step = 0;
%
% LiDAR Range for T2 algorithm
range = 1;
% LiDAR Range for TGF algorithm
rangeTGF = 3;
% This section is to create the sector distribution of the LiDAR scan
K=24;
layer = struct;
layer.environment=[];
% Initiate the obstacle_storage structure
layer.obstacle_storage = [];
layer.obstacle_storage.sector = [];
layer.obstacle_storage.angle = [];
layer.obstacle_storage.distance = [];
layer.obstacle_storage.centre = [];
% set tenacity (0 for left, 1 for right)
tenacity = 0;
% set terminate to 0
terminate = 0;
% Initially the active activeLayer is the first (and only) one
activeLayer = 1;
layerChanged = 0;
% getMode 1 corresponds to MTT, 2 corresponds to BF (explained below).
getMode = 1;
% RemovefromSTM is an array which holds all sectors which need to be
% removed from the short term memory
removefromSTM = [];
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
       [~,scanTGF]=lidarSensor_TGF(obstacleMap,pose,rangeTGF,cur_psi);


%-------------------------------------------------------------------------%
% Behavior
%
%
% check the angle to the goal
[at_waypoint,angleToGoal] = los_auto(cur_x,cur_y,goal);
if at_waypoint == 1
    break;
end 
% run script to plot the sector distribution and obtain target path sector
% Rt
[layer,sensorData,activeLayer,...
    Rt,removefromSTM] = createSectorEnvironment_EG_v2(layer,...
    activeLayer,scan,K,angleToGoal, cur_x,cur_y,outer_loop,removefromSTM);
%
%
% Add the obstacles from Short term memory (STM):
cond = ~isempty(layer(activeLayer).obstacle_storage.sector);
if cond
    [layer(activeLayer)] = addSTMobstacles_EG(layer(activeLayer),K,removefromSTM,Rt);
end

% If the mode is move to target (MTT), i.e. 1
if getMode == 1
    if strcmp(layer(activeLayer).environment.sector(Rt),'allowed') 
        R = Rt;
    else
        getMode = 2; % otherwise set mode to 2, i.e boundary following (BF)
    end
    noLayer = 0;
end
% If the prevEnvironment wasn't assigned (i.e. at first loop or when the
% layer has been cleaned), assign it
if ~exist('prevEnvironment','var') || isempty(prevEnvironment)
   prevEnvironment = layer(activeLayer).environment;
   prevRt = layer(activeLayer).environment.Rt;
end
if getMode == 2
   [R,getMode,activeLayer,layer,noLayer,removefromSTM,prevRt,prevEnvironment] = followBoundary(layer,activeLayer,...
    prevEnvironment,Rt,tenacity,getMode,prevRt,sensorData,scan,...
    angleToGoal,K,outer_loop,cur_x,cur_y,removefromSTM);

end
% if the active layer was the first one and it was emptied -> go to next
% loop
if noLayer == 1
    % create new environment
    [layer,sensorData,activeLayer,...
    Rt,removefromSTM] = createSectorEnvironment_EG_v2(layer,...
    activeLayer,scan,K,angleToGoal, cur_x,cur_y,outer_loop,removefromSTM);
    desired_angle = layer(activeLayer).environment.angle(Rt);
else
 desired_angle = layer(activeLayer).environment.angle(R);
end
% Convert back to the coordinate system of the model (-pi to 0 to pi)
if desired_angle >= pi
    desired_psi = 2*pi - desired_angle;
elseif desired_angle < pi
    desired_psi = -desired_angle;
end
% Add pi/2 to shift the 0 to be on the vertical axis
angleT2 = desired_psi + pi/2;
desired_psi = angleT2;
% [psi_sg,psi_vg,subgoal,virtgoal,gap,closestGap]=TGF_algorithm_EG(obstacleMap,...
%     scanTGF,cur_x,cur_y,angleToGoal,goal,cur_psi,angleT2);
% % psi_sg and psi_vg are in the algorithm FoR so need to convert back to the
% % model one (i.e. just switch the signs around):
% psi_sg = -psi_sg;
% psi_vg = -psi_vg;
% [~,desired_psi] = los_auto(cur_x,cur_y,virtgoal);
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

    
%      V_matrix(1,step) = Vl;
%      V_matrix(2,step) = Vr;

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
    % refresh plot every X steps
    if mod(outer_loop,10)==0
    figure(1);
    clf; show(obstacleMap);grid on; hold on;
    xlabel('X position [m]');
    ylabel('Y position [m]');
    title('Map of the environment');
    drawrobot(0.2,xi(20)+5,xi(19)+5,xi(24),'b');
    goalPlot(1) = plot(goal(2),goal(1),'Marker','x','MarkerFaceColor','blue',...
     'LineWidth',1.5,'MarkerSize',10);
    goalPlot(2) = plot(xio(1,20)+5,xio(1,19)+5,'Marker','x','MarkerFaceColor','red',...
     'LineWidth',1.5,'MarkerSize',10);
    pause(0.001);
%     % draw the sectors 
%         for i=1:1:K
%         drawSectors(layer(activeLayer).environment.angle(i)...
%             ,cur_x,cur_y,layer(activeLayer).environment.sector(i),K,i,R,range)
%         end
    end
 % plot the gaps:
%     if isfield(gap,"Gap")
%         for k=1:1:size(gap,2)
%             if k==closestGap
%                plot(gap(k).Coordinates(:,1),gap(k).Coordinates(:,2),'g');
%             else
%                 if gap(k).Gap == 1
%                      plot(gap(k).Coordinates(:,1),gap(k).Coordinates(:,2),'r');
%                 end
%             end
%         end
%     end
    pause(0.001);
    %----------------------------------------------%
    
end

%----------------------------------------------%
% Plot which points the robot reached
figure(1);
trajectory = plot(xio(:,20)+5,xio(:,19)+5,'k','LineStyle','--','LineWidth',2);
trajectory.Color(4) = 0.5;
legend([goalPlot(1:2),trajectory],{'Target','Start','Path'});

%----------------------------------------------%
toc;
%Plot Variables
% figure(2); plot(xio(:,20),xio(:,19));
% figure(3); plot(xio(:,19));
% figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
