
%----------------------------------------------%
% Workspace Clear up
close all;
clear all;
clc;
%----------------------------------------------%

%----------------------------------------------%
% Setup Simulation
desired_coord(1,:) = [0 3];
desired_coord(2,:) = [1 2];
desired_coord(3,:) = [-1 4];
desired_coord(4,:) = [-1 -2];
desired_coord(5,:) = [-0.2 2];
sim_time = 100;
dT = 0.05;
point = 1;
xi = zeros(1,24); % intial state for x
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
% add timer
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
    
    % get required heading for desired position
    [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_coord,point);
    % if at waypoint and if velocity is sufficiently low (stopped)
    if at_waypoint == 1 && xi(13) <= 0.1   
       robot_path(point,:) = [cur_x,cur_y];
       if point < length(desired_coord)
        point = point+1;
       else
        break;
       end
    end
    % Once we've obtained the desired heading a controller needs to be
    % designed to feed appropriate voltages into the motors
    %
    % First calculate error err_psi between current and desired heading,
    % then error between current and desired distance

    err_psi(n) = desired_psi - cur_psi;

    err_xy(n) = pdist([desired_coord(point,:);cur_x,cur_y],'euclidean');

    % then calculate necessary inputs for heading and velocity using
    % PID control;
    Kp_psi = 20;   
    Ki_psi = .1;
    Kd_psi = .01;

    Kp_xy = 5;   
    Ki_xy = .1;
    Kd_xy = 1;
 
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
    % Clamping Anti-Windup:
    %
    % if input due to distance error is equal to saturation voltage
    if u_xy(n) == 7.2
        % and if the input and the error are both positive or negative
        if sign(err_xy(n)) == sign(u_xy(n))
            err_xy_i(n) = 0;
        end
    end
    % and the same for the u_psi
    if u_psi(n) == 7.2
        % and if the input and the error are both positive or negative
        if sign(err_psi(n)) == sign(u_psi(n))
            err_psi_i(n) = 0;
        end
    end
    
        
    % and then convert them into voltages:

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
    xlabel('y, m'); ylabel('x, m');  
    plot(wall(:,1),wall(:,2),'k-');
    plot(wall2(:,1),wall2(:,2),'k-');
    pause(0.001);
    %----------------------------------------------%
    
end
%----------------------------------------------%
% Plot which points the robot reached and path
figure(1);
for i=1:1:length(desired_coord)
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
