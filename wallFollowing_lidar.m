function [desired_psi,state,stopRobot,originalPosition,stateChanged]=wallFollowing_lidar(objectDetected,...
    state,cur_psi,desired_psi,position,originalPosition,distance,stateChanged)
if abs(abs(cur_psi - desired_psi)-2*pi)<0.05
    desired_psi = cur_psi;
end
% state 0 is neutral wall-following state
% state 1 is turn right until you find a wall to the left
% state 2 is turn 90 degrees left and move forward a bit


if state == 1
    if abs(cur_psi - desired_psi) < 0.05 
        state=0;

    else
        stopRobot=1;

    end
elseif state == 2
    if abs(cur_psi- desired_psi) < 0.05
        if objectDetected(1) == 1 && distance(1,3) < 0.8
            stopRobot=1;
            state=0;
            
        
        elseif abs(position(1) - originalPosition(1)) < 1 &&...
                abs(position(2) - originalPosition(2)) < 1
            desired_psi = cur_psi;
            stopRobot=0;
            
        else          
            state = 0;
           
        end
    else
        stopRobot=1;

    end
elseif state == 0
    if objectDetected(1)==0 && objectDetected(2)==0
        desired_psi = cur_psi - pi/2;
        originalPosition = position;
        state = 2;
        stopRobot=1;
        stateChanged = 1;
    elseif (objectDetected(2) == 0 || all(objectDetected))&& distance(1,3)<0.8 % if there's no left wall
        desired_psi = cur_psi + pi/2;  
        state = 1;  
        stopRobot=1;
        stateChanged = 0;
    elseif objectDetected(1) == 0 % if there's no front wall  
            desired_psi = cur_psi;
            state = 0;    
        stateChanged = 0;
    end
end
if state==0
    stopRobot = 0;
end
end