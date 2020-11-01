function [desired_psi]=objectDetection(cur_psi)
    % while object is being detected, turn right by 0.3 radians
    % until you can't see object anymore
    desired_psi = turnRight(cur_psi,0.3);
   end
