function [desired_psi,state,stopRobot,originalPosition]=wallFollowing_v3(objectDetected,...
    state,cur_psi,desired_psi,position,originalPosition,distance,scan)
if abs(abs(cur_psi - desired_psi)-2*pi)<0.05
    desired_psi = cur_psi;
end
if objectDetected(2) == 1
    parallel = abs(scan(1,2).Cartesian(1,1)) - abs(scan(1,2).Cartesian(end,1));
    isParallel =  abs(parallel) < 0.1;
else
    isParallel = 1;
end
if state == 0
    
    if isParallel == 0
       desired_psi = cur_psi - sign(parallel)*0.1;
       state = 0;
       originalPosition = position;
       stopRobot = 1;
    elseif abs(position(1) - originalPosition(1)) > 0.2 ||...
        abs(position(2) - originalPosition(2)) > 0.2
        if objectDetected(2) == 0 % is left free
            desired_psi = cur_psi - pi/2;
            originalPosition = position;
            state = 1;
            stopRobot = 1;
        else
            if objectDetected(1) == 0 || distance(1,3) > 0.8% if there is no front wall
                desired_psi = cur_psi;
                originalPosition = position;
                state = 0;
                stopRobot = 0;
            else
                if objectDetected(3) == 0 % if the right is free
                    state = 2;
                    desired_psi = cur_psi + pi/2;
                    originalPosition = position;
                    stopRobot = 1;
                else
                    desired_psi = cur_psi + pi;
                    originalPosition = position;
                    state = 0;
                    stopRobot = 1;
                end
            end
        end
    else % if robot hasnt traversed necessary distance
         desired_psi = cur_psi;
         state = 0;
         stopRobot = 0;
    end
elseif state == 1
    if abs(cur_psi- desired_psi) < 0.05
        state = 0;
        originalPosition = position;    
        stopRobot = 0;
    else
        stopRobot = 1 ;
    end
elseif state == 2
    if abs(cur_psi- desired_psi) < 0.05
        state = 0;
        originalPosition = position;
        stopRobot = 0;
    else
        stopRobot = 1;
    end
end
        
end
