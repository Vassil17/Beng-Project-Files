% Simple Wall following Algorithm
function [stopRobot,desired_psi,followWall]=wallFollowing(objectDetected,distance,...
                                            cur_psi,followWall)


if objectDetected(1)==1
   stopRobot = 1;   
   % turn robot to the right by a bit
   desired_psi = turnRobot(cur_psi,0.25); 
% if robot isn't parallel to wall   
elseif abs(distance(2,1) - distance(2,2)) > 0.2
   stopRobot = 1;   
   % find which direction to rotate in to get parallel
   rotDir = sign(distance(2,1) - distance(2,2));
   % turn robot until parallel
   desired_psi = turnRobot(cur_psi,rotDir*0.2); 
elseif abs(distance(2,1) - distance(2,2)) <= 0.2 
    stopRobot = 0;   
    % find which direction to rotate in to get parallel
    rotDir = sign(distance(2,1) - distance(2,2));
    % turn robot until parallel
    desired_psi = turnRobot(cur_psi,rotDir*0.1); 
elseif all(objectDetected(2:end)) % robot needs to follow wall:
    stopRobot = 0;
    desired_psi = cur_psi;

% if the wall disappears, it means it either made a 90 degree turn or
% is gone:
elseif followWall == 1 && ~all(objectDetected(2:end))
   stopRobot = 1;
   % turn robot 90 degrees left
   desired_psi = turnRobot(cur_psi,-1.5708); 
end
% tell robot it needs to follow wall
followWall = 1;
end
