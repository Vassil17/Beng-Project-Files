function [desired_psi,state,stopRobot,originalPosition,check_for_goal]=wallFollowing_updated(objectDetected,...
    state,cur_psi,desired_psi,position,originalPosition,distance,check_for_goal,scan)
if abs(abs(cur_psi - desired_psi)-2*pi)<0.05
    desired_psi = cur_psi;
end
% state 0 is neutral wall-following state
% state 1 is turn right until you find a wall to the left
% state 2 is turn 90 degrees left and move forward a bit


if state == 1
    if abs(cur_psi - desired_psi) < 0.05 
        state=0;
        check_for_goal = 0;
    else
        stopRobot=1;

    end
elseif state == 2
    
    if abs(cur_psi- desired_psi) < 0.05
        if objectDetected(1) == 1 && distance(1,3) < 0.8
            stopRobot=1;
            state=0;
            check_for_goal = 1;
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
    check_for_goal = 0;
    if objectDetected(1)==0 && objectDetected(2)==0
        desired_psi = cur_psi - pi/2;
        originalPosition = position;
        state = 2;
        stopRobot=1;      
    elseif (objectDetected(2) == 0 || all(objectDetected))&& distance(1,3)<0.8 % if there's no left wall
        desired_psi = cur_psi + pi/2;  
        state = 1;  
        stopRobot=1;
    elseif objectDetected(1) == 0 % if there's no front wall  
        parallel_err = (scan(1,2).Cartesian(1,2) - scan(1,2).Cartesian(end,2));
        if  abs(parallel_err) > 0.1
            desired_psi = cur_psi + sign(parallel_err)*0.01;   
        else
            desired_psi = cur_psi;
        end
        state = 0;    
    end
end
if state==0
    stopRobot = 0;
end
end