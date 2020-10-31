% Version 3 - Alternative
%
% Change robot so that it determines heading before moving
% and check performance
%
%----------------------------------------------%
% Workspace Clear up
close all;
clear all;
clc;
%----------------------------------------------%

%----------------------------------------------%
% Setup Simulation
desired_coord(1,:) = [-3 0];
desired_coord(2,:) = [0 0];
desired_coord(3,:) = [-1 4];
desired_coord(4,:) = [-1 -2];
desired_coord(5,:) = [-0.2 2];
sim_time = 100;
dT = 0.05;
point = 1;
xi = zeros(1,24); % initial state for x
LeftS = 0;
RightS = 0;
err_psi_i(1) = 0;
err_xy_i(1) = 0;

%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;

Obs_Matrix = zeros(max_x/0.01,max_y/0.01);

wall = WallGeneration1(-1, 1,1.2,1.2,'h');
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

    % Check the sensors for objects:
    [objectDetected] = obstacleSensor(cur_psi,cur_x,cur_y,max_x,max_y,Obs_Matrix);
    if objectDetected == 1
        [objectDetected,distance_min] = obstacleSensor(cur_psi,cur_x,cur_y,max_x,max_y,Obs_Matrix);
        fprintf('Object Detected in %.2f \n',distance_min);
    end
    % get required heading for desired position
    [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_coord,point);
    % if at waypoint and if velocity is sufficiently low (i.e. robot has stopped)
    if at_waypoint == 1 && xi(13) <= 0.01
       robot_path(point,:) = [cur_x,cur_y];
       disp(n);
       if point < size(desired_coord,1)
        % look at the next desired point
        point = point+1;
        [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_coord,point);
       else
        break;
       end
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
    err_xy(n) = pdist([desired_coord(point,:);cur_x,cur_y],'euclidean');
    
    %---------------------------------------------------------------------%
    
    % PID Controllers for heading and distance:
    
    Kp_psi = 25;   
    Ki_psi = .1;
    Kd_psi = .01;

    Kp_xy = 10;  % 6
    Ki_xy = .1; %.1
    Kd_xy = 6;  % 2
 
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
    % For error in xy:
    err_xy_i(n) = err_xy_i(prev_n)  + err_xy(n)*dT;
    err_xy_d(n) = (err_xy(n) - err_xy(prev_n))/dT;
    
    u_xy(n) = Kp_xy*err_xy(n)+Ki_xy*err_xy_i(n)+Kd_xy*err_xy_d(n);

    % While the heading is wrong, don't move robot
    if abs(err_psi(n)) > 0.1
        u_xy(n) = 0;
    end
    %------------------------------------------------------------------%
    % Convert inputs into voltages:
    
    Vl = (u_xy(n) + u_psi(n))/2;
    Vr = (u_xy(n) - u_psi(n))/2;
    
    
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
    drawSensorCone(xi(24),xi(19),xi(20),1);
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
%----------------------------------------------%
