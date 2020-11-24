%
% los_auto
%
% Handels the line of sight autopilot algorithim
%
% Author: Kevin J Worrall
%
% Created: 22/01/08
%
% (Based on original los_auto)
%

function [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_coord,point)
desired_pt = desired_coord(point,:);
% set waypoint
waypoint = desired_pt;

% check to see if in acceptance radius
cur_radius = sqrt(((cur_x-waypoint(1))^2)+((cur_y-waypoint(2))^2));

if (cur_radius < 0.2),
    at_waypoint = 1;
else
    at_waypoint = 0;
end;

% Calculate heading
psi_cal=atan2((waypoint(2)-cur_y),(waypoint(1)-cur_x));



if isnan(psi_cal),
    if (waypoint(2)-cur_y)>=0,
        desired_psi = pi/2;
    else
        desired_psi = -pi/2;
    end;
else
    desired_psi = psi_cal;
end;
%------------------------------------------------------------------------%