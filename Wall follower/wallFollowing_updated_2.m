function [desired_psi,state,stopRobot,originalPosition,check_for_goal]=wallFollowing_updated_2(objectDetected,...
    state,cur_psi,desired_psi,position,originalPosition,distance,check_for_goal,scan)
if abs(abs(cur_psi - desired_psi)-2*pi)<0.05
    desired_psi = cur_psi;
end
% Check for obstacles and categorise them
if objectDetected(1)==1
    if objectDetected(2)==1
        obstacle = "both";
    else
        obstacle = "front";
    end
elseif objectDetected(2)==1
    obstacle='left';
else
    obstacle='none';
end

% state 1 is neutral wall-following state
% state 2 is turn 90 degrees left and move forward a bit
% state 3 is turn 90 degrees left until you find a wall

if state == 2   
    
    if abs(cur_psi- desired_psi) < 0.05
        if abs(position(1) - originalPosition(1)) < 0.3 &&...
                abs(position(2) - originalPosition(2)) < 0.3
            desired_psi = cur_psi;
            stopRobot = 0;
        elseif objectDetected(2)==0
                desired_psi = cur_psi - pi/2;
                stopRobot=1;
                originalPosition = position;
                check_for_goal = 0;        
                state = 2;
        else          
            state = 1;       
        end
    else
        stopRobot=1;

    end
% elseif state == 3
%     
%     stopRobot = 0;
%     if abs(cur_psi- desired_psi) < 0.05
%         if abs(position(1) - originalPosition(1)) >= 0.3 ||...
%                 abs(position(2) - originalPosition(2)) >= 0.3
%             check_for_goal = 0;
%             state=1;
%         end
%     end
elseif state == 1    
    check_for_goal = 0;
    if strcmp(obstacle,'none')
        desired_psi = cur_psi - pi/2;
        originalPosition = position;
        state = 2;
        stopRobot=1;      
    elseif (strcmp(obstacle,'front')||strcmp(obstacle,'both'))&& distance(1,3)<0.5 
        % turn robot slightly to the right and recheck sensors
        desired_psi = cur_psi + 0.5;
        state = 1;  
        stopRobot=1;
    elseif strcmp(obstacle,'left')
        parallel = abs(scan(1,2).Cartesian(1,1)) - abs(scan(1,2).Cartesian(end,1));
        isParallel =  abs(parallel) > 0.1;
        if isParallel == 1
           desired_psi = cur_psi - sign(parallel)*0.1;
        else
            desired_psi = cur_psi;
        end
        state = 1;
    end
end
if state==1
    stopRobot = 0;
end
end