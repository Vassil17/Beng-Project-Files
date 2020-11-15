% Version 5
%
% - Changed robot so that it determines heading before moving.
% - Changed to use velocity control instead of distance control.
% - Uses obstacleSensor to detect obstacles in field of view.
% 
%----------------------------------------------%
% Workspace Clear up
close all;
clear all;
clc;
%----------------------------------------------%

%----------------------------------------------%
% Setup Simulation
desired_coord(1,:) = [0 -4];
% desired_coord(2,:) = [4 0];
% desired_coord(3,:) = [-1 4];
% desired_coord(4,:) = [0 0];
% desired_coord(5,:) = [-0 2];
sim_time = 60;
dT = 0.05;
point = 1;
xi = zeros(1,24); % initial state for x
LeftS = 0;
RightS = 0;
err_psi_i(1) = 0;
err_vel_i(1) = 0;
stopRobot = 0;
moveToAvoid = 0;

%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;


Obs_Matrix = zeros(max_x/0.01,max_y/0.01);
visionMatrix = zeros(max_x/0.01,max_y/0.01);

wall = WallGeneration1(-1, 1,1,1,'h');
wall2 = WallGeneration1(-3, -3, -2, 2,'v');

for x=1:length(wall)
    % xpos = x point of wall  + origin
    xpos = (wall(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall2)
    
    xpos = (wall2(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall2(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
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
    cur_x = xi(19);
    cur_y = xi(20);
    cur_psi = xi(24);
    cur_vel = xi(13);
    n = outer_loop;
    % Set up sensor position (x,y) for each sensor for psi=0;
    sensors = [0.1 0.1;0.1 -0.1;0 -0.1;0 0.1];
    % Set up angle for each sensor
    sensorAngle = [cur_psi,cur_psi,(cur_psi-pi/2),(cur_psi+pi/2)];
    
    % Create each sensor:
    for i=1:length(sensors)
       % use rotation matrix to find sensor position based on heading
       sensors(i,:) = transpose(([cos(cur_psi), -sin(cur_psi);sin(cur_psi), cos(cur_psi)]*sensors(i,:)'));
       objectDetected(i) = obstacleSensor(sensorAngle(i),cur_x,cur_y,sensors(i,:),...
           max_x,max_y,Obs_Matrix,visionMatrix);
       if objectDetected(i) == 1
           [~,distance] = obstacleSensor(sensorAngle(i),cur_x,cur_y,sensors(i,:),...
               max_x,max_y,Obs_Matrix,visionMatrix);
           fprintf('Object detected by sensor %i at %i\n',i,distance);
       end
    end

    % if object is detected and distance is small
    if any(objectDetected) && distance < 0.8
           % stop robot;
           stopRobot = 1;
           [desired_psi] = objectDetection(cur_psi);
           moveToAvoid = 1;
           loc = [cur_x cur_y];
    elseif moveToAvoid == 1
        stopRobot = 0;
        % if robot has moved some distance to avoid object, go back to
        % going to desired location
        if sqrt((cur_x - loc(1))^2 + (cur_y - loc(2))^2) > 1.5
            moveToAvoid = 0;
        end
        
    else
        % otherwise go to desired location
        [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_coord,point);
    end
    
    if at_waypoint == 1 && cur_vel <= 0.01
       robot_path(point,:) = [cur_x,cur_y];
       break;
    end
       
    %---------------------------------------------------------------------%
    % Once we've obtained the desired heading a controller needs to be
    % designed to feed appropriate voltages into the motors
    %
    % First calculate error err_psi between current and desired heading,
    % then error between current and desired distance
    
    % Change heading to be from 0 to 360 degrees
    if cur_psi < 0 
        cur_psi_360 = cur_psi + 2*pi;
    elseif cur_psi >= 2*pi
        cur_psi_360 = cur_psi - 2*pi;
    else  
        cur_psi_360 = cur_psi;
    end
    if desired_psi < 0
        desired_psi_360 = desired_psi + 2*pi;
    elseif desired_psi >= 2*pi
        desired_psi_360 = desired_psi - 2*pi;
    else
        desired_psi_360 = desired_psi;
    end
    
    err_psi(n) = desired_psi - cur_psi_360;
    % error shouldn't be bigger than 180 degrees:
    if abs(err_psi(n)) > pi
        err_psi(n)=-sign(err_psi(n))*(2*pi-abs(err_psi(n)));
    end
    % If robot has been told to stop or heading is 0, set vel to 0
    if stopRobot == 1 || abs(err_psi(n)) > 0.2
        desired_vel = 0;
    elseif moveToAvoid ==1
        desired_vel = 1;
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
    clf; hold on; grid on; axis([-5,5,-5,5]);
    drawrobot(0.2,xi(20),xi(19),xi(24),'b');
    for i=1:length(sensors)
        drawSensorCone(sensorAngle(i),xi(19)+sensors(i,1),xi(20)+sensors(i,2),1);
    end
    xlabel('y, m'); ylabel('x, m');  
    plot(wall(:,1),wall(:,2),'k-');
    plot(wall2(:,1),wall2(:,2),'k-');
    pause(0.001);
    %----------------------------------------------%
    
end
%----------------------------------------------%
% Plot which points the robot reached
figure(1);
for i=1:1:size(desired_coord,1)
    plot(robot_path(i,2),robot_path(i,1),'-x');
    plot(xio(:,20),xio(:,19),'k');
end
%----------------------------------------------%
toc;
%Plot Variables
figure(2); plot(xio(:,20),xio(:,19));
figure(3); plot(xio(:,19));
figure(4); plot(xio(:,24));
%figure(5);hold on; plot(xio(:,13));
%----------------------------------------------%
