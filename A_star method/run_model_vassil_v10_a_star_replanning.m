% Version 10 A* with re-planning;
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
sim_time = 60;
dT = 0.05;
point = 1;
xi = zeros(1,24); % initial state for x
xi(19) = -4;
xi(20) = 1;
xi(24) = pi/2;
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
desired_vel = 0;
step = 0;
%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;
resolution = 10;
%
%
dontRecheck = 0;
% Choose scenario (start and goal defined in scenarios)
scenario = 13; % 12 is the estimated version of 6
scenario_updated = 8; 
[obstacleMap,start,goal]=mapEnvironments(resolution,scenario_updated);
[estimatedMap,~]=mapEnvironments(resolution,scenario);
[plotObstacleMap,~]=mapEnvironments(resolution,scenario_updated);
%
%
%
%start(2) = start(2) - 0.5; 
%
%
xi(19) = start(1) - 5;
xi(20) = start(2) - 5;
goal_path = [goal(2) goal(1) pi/2]; %for scenario 10
%goal_path = [13.8 6 pi];
start_path = [start(2) start(1) pi/2];
% Inflate the obstacles:
% inflate(obstacleMap,0.1);
% inflate(estimatedMap,0.1);
% Create range sensor
obstacleSensor = rangeSensor('Range',[0 2],'HorizontalAngle',[-pi pi],...
    'HorizontalAngleResolution',0.01);
numReadings = obstacleSensor.NumReadings;
startTime = [];
endTime = [];
counterPlan = 0;
%----------------------------------------------%
startTime = tic;
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
       

     % Set up sensor position (x,y) for each sensor for psi=0;
    sensors = [cur_y cur_x; cur_y cur_x;];
    % Set up angle for each sensor
    sensorAngle = [cur_psi; cur_psi - pi/2];
      
    

%-------------------------------------------------------------------------%
% Behavior - A* pathplanning
%
% Save current cputime
% Initially, create the plan:
    
angle(cur_psi<0) = -cur_psi;
angle(cur_psi>0) = 2*pi - cur_psi;
angle(cur_psi==0) = cur_psi;

    if current_point == 0
        algTime = tic;
        % Define algorithm to use for pathfinding
        % Create validator
        validator = validatorOccupancyMap;
        validator.Map = estimatedMap;
        planner = plannerHybridAStar(validator,'MinTurningRadius',0.64);
        %planner.MotionPrimitiveLength = 0.25;
        planner.DirectionSwitchingCost = 1;
        storePlanner(1) = planner;
        % Create plan based on global map
        path = plan(planner,start_path,goal_path);
        % Create points for robot to follow
        path=path.States;
        startPoses = path(1:end-1,:);
        endPoses = path(2:end,:);
        poses = [];
        for i=1:size(startPoses,1)
           [xx,yy]=straightLine(startPoses(i,1:2),endPoses(i,1:2),100);
           pose = [xx' yy'];
           poses = [poses; pose];
        end
        endTime(end+1) = toc(algTime);
        storeCnt = 1;
        pathStorage = struct;
        pathStorage(storeCnt).Path = [poses];
        poses_inverted = poses;
        poses=[poses(:,2),poses(:,1)];
        current_point = current_point + 1;
    else
        % check sensor reading and
        % insert sensor reading into the estimatedMap
        [ranges, angles] = obstacleSensor([cur_y cur_x cur_psi], obstacleMap);
        insertRay(estimatedMap, [cur_y cur_x cur_psi], ranges, angles, ...
        obstacleSensor.Range(end));
         % drawnow;
        check = any(checkOccupancy(estimatedMap,poses_inverted(:,1:2)));
        if check
           counterPlan=counterPlan+1;    
        end
        if check && ~dontRecheck && mod(counterPlan,20)==0
            replanTime = tic;
            current_point = 1;
            storeCnt = storeCnt+1;   
            validator.Map = estimatedMap;
            planner = plannerHybridAStar(validator,...
                'MinTurningRadius',.64,'ReverseCost',8);
            planner.NumMotionPrimitives = 5;
            planner.MotionPrimitiveLength = 0.1;
            planner.DirectionSwitchingCost = 1;
            storePlanner(end+1) = planner;
            % Create plan
            path = plan(planner,[cur_y cur_x angle+pi/2],goal_path);
            % Create points for robot to follow
            path=path.States;
            startPoses = path(1:end-1,:);
            endPoses = path(2:end,:);   
            poses = [];
           for i=1:size(startPoses,1)
               [xx,yy]=straightLine(startPoses(i,1:2),endPoses(i,1:2),100);
               pose = [xx' yy'];
               poses = [poses; pose];
           end
           pathStorage(storeCnt).Path = [poses];
            poses_inverted = poses;
            poses=[poses(:,2),poses(:,1)];
           endTime(end+1) = toc(replanTime);
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
        [desired_vel] = getVelocity(goal(point,:),cur_x,cur_y);
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
    
    %----------------------------------------------%
    if mod(outer_loop,10)==0
    figure(1);
    clf; show(plotObstacleMap);grid on; hold on;
    drawrobot(0.2,xi(20)+5,xi(19)+5,xi(24),'b');
    goalPlot(1) = plot(goal(2),goal(1),'Marker','x','MarkerFaceColor','blue',...
     'LineWidth',1.5,'MarkerSize',10);
    goalPlot(2) = plot(xio(1,20)+5,xio(1,19)+5,'Marker','x','MarkerFaceColor','red',...
     'LineWidth',1.5,'MarkerSize',10);
    
    for i=1:20:size(poses,1)
       plot(poses(:,2),poses(:,1));
    end
    
    end

    pause(0.001);
    %----------------------------------------------%
    
end
runtime = toc(startTime);
%----------------------------------------------%
% Plot which points the robot reached
figure(1);
trajectory = plot(xio(:,20)+5,xio(:,19)+5,'k','LineStyle','--','LineWidth',2);
trajectory.Color(4) = 0.5;
legend([goalPlot(1:2),trajectory],{'Target','Start','Path'});
%----------------------------------------------%
%Plot Variables
% figure(2); plot(xio(:,20),xio(:,19));
% figure(3); plot(xio(:,19));
% figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
