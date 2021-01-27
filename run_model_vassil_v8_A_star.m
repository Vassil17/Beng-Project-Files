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
desired_coord(1,:) = [7 5];
sim_time = 60;
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
originalPosition = 0;

%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;
resolution = 10;
% Create the obstacle map
obstacleMap = binaryOccupancyMap(max_x,max_y,resolution);


% walls are now contained in a cell

wall{1} = WallGeneration1(7,7,6,8,'v');
wall{2} = WallGeneration1(2,7,6,6,'h');
wall{3} = WallGeneration1(2,7,8,8,'h');

% % wall{6}
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
% Initialise the counter for following the path
current = 1;
obstacleMatrix = occupancyMatrix(obstacleMap);
start=[50,50];
ending=[60,60];
path = Astar(obstacleMatrix, start, ending);
for i=1:size(path,2)
    path{i}=path{i}/10;
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
      
    
    % Create each sensor:
    for i=1:size(sensors,1)
        % rotation matrix isnt needed for now as sensors are at robot
        % centre
%       % use rotation matrix to find sensor position based on heading
%       sensors(i,:) = transpose(([cos(cur_psi), -sin(cur_psi);sin(cur_psi), cos(cur_psi)]*sensors(i,:)'));
       pose = [sensors(i,:)];
       
       [obstacleMap,scan(i),distance(i,:),objectDetected(i)]=lidarSensor(obstacleMap,pose,sensorAngle(i));
    end
    if distance(1,3) < 1
        fprintf('Object too close! At %.2f m\n',distance(1));
    end

%-------------------------------------------------------------------------%
% Behavior
%
 position = [cur_y,cur_x];
 goal=path{current};
 if ~any((position-goal)<0.05)
     current=current+1;
     goal=path{current};
 else
     [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,goal); 
 end

  
%-------------------------------------------------------------------------%    

    %---------------------------------------------------------------------%
    % Once we've obtained the desired heading a controller needs to be
    % designed to feed appropriate voltages into the motors
    %
    % First calculate error err_psi between current and desired heading,
    % then error between current and desired distance
        

    % Change heading to be from 0 to 360 degrees
    
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
        [desired_vel] = getVelocity(desired_coord(point,:),cur_x,cur_y);
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
    clf; hold on; grid on; show(obstacleMap);
    drawrobot(0.2,xi(20)+5,xi(19)+5,xi(24),'b');
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
%----------------------------------------------%
% Plot which points the robot reached
figure(1);
for i=1:1:size(desired_coord,1)
    plot(robot_path(i,2),robot_path(i,1),'-x');
    plot(xio(:,19),xio(:,20),'k');
end
%----------------------------------------------%
toc;
%Plot Variables
% figure(2); plot(xio(:,20),xio(:,19));
% figure(3); plot(xio(:,19));
% figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
