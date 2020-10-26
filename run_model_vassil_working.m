
%----------------------------------------------%
% Workspace Clear up
close all;
clear all;
clc;
%----------------------------------------------%

%----------------------------------------------%
% Setup Simulation
desired_pt = [-1 1];
sim_time = 30;
dT = 0.05;
xi = zeros(1,24); % intial state for x
LeftS = 0;
RightS = 0;
err_psi_i(1) = 0;
err_dist_i(1) = 0;
%----------------------------------------------%

%----------------------------------------------%
% Create Environment
max_x = 10;
max_y = 10;

Obs_Matrix = zeros(max_x/0.01,max_y/0.01);

wall = WallGeneration1(-1, 1,1.2,1.2,'h');
wall2 = WallGeneration1(-3, -3, -2, 2,'v');

for x=1:length(wall)
    
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
    
    % get required heading for desired position
    [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_pt);
    % If close enough to desired location, stop spinning the robot
    if at_waypoint == 1
        desired_psi = cur_psi;
    end
    % Once we've obtained the desired heading a controller needs to be
    % designed to feed appropriate voltages into the motors
    %
    % First calculate error err_psi between current and desired heading,
    % then error between current and desired distance
    err_psi(n) = desired_psi - cur_psi;
    err_dist(n) = pdist([desired_pt;cur_x,cur_y],'euclidean');

    
    
    % then calculate necessary inputs for heading and velocity using
    % PID control;
    Kp_psi = 30;
    Ki_psi = 10;
    Kd_psi = 0.4;

    Kp_dist = 10;
    Ki_dist = .01;
    Kd_dist = 1;
 
    if n == 1
        prev_n = 1;
    else
        prev_n = n-1;
    end
    % Using Euler's backward rule
    err_psi_i(n) = err_psi_i(prev_n)  + err_psi(n)*dT;
    err_dist_i(n) = err_dist_i(prev_n) + err_dist(n)*dT;
    
    u_psi(n) = Kp_psi * err_psi(n) + Ki_psi * err_psi_i(n) +...
        + Kd_psi * (err_psi(n) - err_psi(prev_n))/dT;
    u_dist(n) = Kp_dist * err_dist(n) + Ki_dist * err_dist_i(n) +...
        + Kd_dist * (err_dist(n) - err_dist(prev_n))/dT;

    
    % and then convert them into voltages:
    Vl = (u_dist(n) + u_psi(n))/2;
    Vr = (u_dist(n) - u_psi(n))/2;
    if abs(Vl) >= 50
        Vl = sign(Vl)*50;
    end
    if abs(Vr) >= 50
        Vr = sign(Vr)*50;
    end
    Vl = interp1([-50,50],[-7.5,7.5],Vl);
    Vr = interp1([-50,50],[-7.5,7.5],Vr);
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
    xlabel('y, m'); ylabel('x, m');
    plot(wall(:,1),wall(:,2),'k-');
    plot(wall2(:,1),wall2(:,2),'k-');
    pause(0.001);
    %----------------------------------------------%
    
end
%----------------------------------------------%

%----------------------------------------------%
%Plot Variables
figure(2); plot(xio(:,20),xio(:,19));
figure(3); plot(xio(:,19));
figure(4); plot(xio(:,24));
%----------------------------------------------%
