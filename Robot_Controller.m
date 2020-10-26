% Robot_Controller
%
%
function [at_waypoint,desired_psi,Vl,Vr] = Robot_Controller(xi,desired_pt,outer_loop,dT)
cur_x = xi(19);
cur_y = xi(20);
cur_psi = xi(24);
n = outer_loop;
cur_vel = xi(13);

% get required heading for desired position
[at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_pt);

% Once we've obtained the desired heading a controller needs to be
% designed to feed appropriate voltages into the motors
%
% First calculate error err_psi between current and desired heading,
% err_distance and err_vel
err_psi(n) = desired_psi - cur_psi;
err_dist(n) = sqrt((desired_pt(1)-cur_x)^2 + (desired_pt(2)-cur_y)^2);

if err_psi(n) <= 0.001
    desired_vel = 2*err_dist;
else
    desired_vel = 0;
end
err_vel = desired_vel - cur_vel;

% then calculate necessary inputs for heading and velocity using
% PID control;
Kp_psi = 15;
Ki_psi = 0;
Kd_psi = 0;

Kp_dist = 10;
Ki_dist = 0;
Kd_dist = 0;

if n == 1
    prev_n = 1;
else
    prev_n = n-1;
end

u_psi = Kp_psi * err_psi(n) + Ki_psi * sum(err_psi) +...
    Kd_psi * (err_psi(n) - err_psi(prev_n))/dT;
% u_dist = Kp_dist * err_dist(n) + Ki_dist * sum(err_dist) +...
%     Kd_dist * (err_dist(n) - err_dist(prev_n))/dT;

u_vel = 5;
% and then convert them into voltages:
Vl = (u_vel + u_psi)/2;
Vr = (u_vel - u_psi)/2;


end