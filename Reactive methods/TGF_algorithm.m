function [psi_sg,psi_vg,subgoal,virtgoal]=TGF_algorithm(scan,cur_x,cur_y,angleToGoal,goal,cur_psi)
% Angles in this algorithm are defined with 0 at the current heading
% (cur_psi) and positive for (0,pi] (ccw) and negative for (0,-pi) (cw);
%
%
% LiDAR Angles are defined the same way in lidarSensor_TGF;

% First check continuity of detected obstacles in order to find gaps
i=1;
gap = struct;
% R is robot radius and Ds is security zone for obstacle avoidance
R = 0.1;
Ds = R+0.1;
for index=1:1:length(scan.Angles)
    % There might be a gap between the final and first point so need this
    % if condition to essentially make the array cyclical:
    if index == length(scan.Angles)
      if abs(scan.Angles(index) - scan.Angles(1)) >= 0.1
        
        gap_start(i,:) = scan.Cartesian(index,:) + [cur_y cur_x];
        gap_end(i,:) = scan.Cartesian(1,:) + [cur_y cur_x];
        i=i+1;
      end    
    else
        if abs(scan.Angles(index) - scan.Angles(index+1)) >= 0.1

            gap_start(i,:) = scan.Cartesian(index,:) + [cur_y cur_x];
            gap_end(i,:) = scan.Cartesian(index+1,:) + [cur_y cur_x];
            i=i+1;
        end
     end
end
if  i==1 % i.e. if there are no gaps
    psi_sg = 0;
    psi_vg = 0;
else
    for gapNumber = 1:i-1
       x=[gap_start(gapNumber,1) gap_end(gapNumber,1)];
       y=[gap_start(gapNumber,2) gap_end(gapNumber,2)];
       diff = x(2) - x(1);
       if diff == 0
          x(2) = x(1) + 0.01;
          diff = x(2) - x(1);
       end
          x_i = x(1):(diff/10):x(2);

       y_i = interp1(x,y,x_i,'linear');


       plot(x_i,y_i,'HandleVisibility','off');
       gap(gapNumber).Coordinates = [x_i' y_i'];
       gap(gapNumber).Width = sqrt((x_i(1)-x_i(end))^2 + (y_i(1)-y_i(end))^2);

       if gap(gapNumber).Width < 0.5
          gap(gapNumber).Gap = 'blocked';
          gap(gapNumber) = [];
          continue;
       else
           gap(gapNumber).Gap = 'allowed';
       end

       % Find the smallest angle to each gap (either to its beginning or end)
       % And then calculate the Angle between each gap and the goal line.
       gap(gapNumber).Centre = [x_i(ceil(end/2)) y_i(ceil(end/2))];
       % SideAngle(1) is right side angle, 2 is left side angle
       [~,SideAngle(1)] = los_auto(cur_x,cur_y,[y_i(1) x_i(1)]);
       [~,SideAngle(2)] = los_auto(cur_x,cur_y,[y_i(end) x_i(end)]);
       % Need angles to be relative to the current heading and in the algorithm
       % notation (clockwise from heading is negative)
       for sideIndex = 1:length(SideAngle)
    %       if sign(cur_psi)==sign(SideAngle(sideIndex))
    %       elseif sign(cur_psi)~=sign(SideAngle(sideIndex))
    %           SideAngle(sideIndex) = -(cur_psi - SideAngle(sideIndex));
    %       elseif cur_psi <= SideAngle(sideIndex) &&...
    %               sign(cur_psi)~=sign(SideAngle(sideIndex))
    %           
              SideAngle(sideIndex) = cur_psi - SideAngle(sideIndex);
    %       end
       end
       % Sides stores the right side (row 1) and the left side (row 2) of gap
       gap(gapNumber).Sides(1,:) = [x_i(1) y_i(1) SideAngle(1)];
       gap(gapNumber).Sides(2,:) =[x_i(end) y_i(end) SideAngle(2)];
       index = find(abs(gap(gapNumber).Sides(:,3))==min(abs(SideAngle(1)),abs(SideAngle(2))));
       gap(gapNumber).ClosestGapAngle = gap(gapNumber).Sides(index,3);
       gap(gapNumber).AngleDifference = abs(cur_psi -angleToGoal - gap(gapNumber).ClosestGapAngle);
    end
    % Angle to the goal should be relative to current heading:
    relativeAngleToGoal = cur_psi - angleToGoal;
    % Find which gap has the smallest angle to the goal
    closestGap = find([gap.AngleDifference] == min([gap.AngleDifference]));

    % Theta_cs is the closest gap side angle
    theta_cs = gap(closestGap).ClosestGapAngle;

    % Theta_os is the other gap side angle
    index_theta_os = find((gap(closestGap).Sides(:,3) ~= theta_cs));
    theta_os = gap(closestGap).Sides(index_theta_os,3);
    % r_cs is the distance to the theta_cs gap side
    r_cs = sqrt((cur_y - gap(closestGap).Sides(index_theta_os,1))^2+...
        (cur_x - gap(closestGap).Sides(index_theta_os,2))^2);

    if  theta_cs == gap(closestGap).Sides(2,3) % if the left side is closest
        theta_mid = theta_cs - (theta_cs - theta_os)/2;
        theta_scs = theta_cs  - asin((R+Ds)/r_cs);
    else
        theta_mid = theta_cs + (theta_os - theta_cs)/2;
        theta_scs = theta_cs  + asin((R+Ds)/r_cs);
    end
    if (angleToGoal) <= (cur_psi - theta_cs) && (angleToGoal) > (cur_psi - theta_os)
        psi_sg = 0;
    elseif abs(theta_cs - theta_mid) < abs(theta_cs - theta_scs)
        % Psi_sg is the angle to the subgoal    
        psi_sg = theta_mid - relativeAngleToGoal;
    else
        psi_sg = theta_scs - relativeAngleToGoal;
    end

     % Virtual goal and its associated angle show by how much the psi_sg should
     % be rotated in order to safely avoid obstacles:

     % Define beta as the angle towards the closest obstacle:
     if psi_sg ~= angleToGoal
        angleToTarget = relativeAngleToGoal+psi_sg;
     else
         angleToTarget = relativeAngleToGoal;
     end
     indx = find(scan.Ranges == min(scan.Ranges));
     dist = scan.Cartesian(indx,:)+[cur_y cur_x];
     [~,globalBeta] = los_auto(cur_x,cur_y,[dist(2) dist(1)]);
     beta = cur_psi-globalBeta;
     % Define alpha as the angle to the goal
     alpha = angleToTarget;
     %

     gamma = alpha - beta;
     %
     if beta==0 || alpha==0
         beta = beta+0.0001;
         alpha=alpha+0.0001;
     end
     if abs(alpha-beta)>pi/2
         psi_vg = 0;
     elseif abs(gamma)<pi && sign(alpha)~=sign(beta)
         psi_vg = -sign(beta)*pi/2 - gamma;
     elseif abs(gamma)>=pi && sign(alpha)~=sign(beta)
        psi_vg = -sign(beta)*3*pi/2 - gamma;
     elseif abs(beta)>=abs(alpha) && sign(alpha)==sign(beta)
        psi_vg = -sign(beta)*pi/2 - gamma;
     elseif abs(beta)<abs(alpha) && sign(alpha)==sign(beta)
        psi_vg = sign(beta)*pi/2 - gamma;
     end
end
% Need to rotate the vector to the goal by psi_sg for the subgoal and by
% additional psi_vg for the virtual goal
% d is the magnitude of the goal vector
d = sqrt((goal(1)-cur_x)^2 + (goal(2)-cur_y)^2);
% then need to find angleToGoal in the standard coordinate system
if angleToGoal <= 0
    goalAngle = -angleToGoal;
elseif angleToGoal >0 && angleToGoal <=pi
    goalAngle = 2*pi - angleToGoal;
end
% add pi/2 because the 0 should be on the +ve x axis rather than +y axis
goalAngle = goalAngle + pi/2;
v_goal = [d*cos(goalAngle);d*sin(goalAngle)];
% Rotate v_goal by psi_sg to obtain v_sg (subgoal)
% But psi_sg is defined relative to current heading so convert:
%psi_sg = cur_psi + psi_sg;
v_sg = [cos(psi_sg) -sin(psi_sg); sin(psi_sg) cos(psi_sg)]*v_goal;
subgoal = [v_sg(2)+cur_x v_sg(1)+cur_y];
% Then analogous with the virtual goal
v_vg = [cos(psi_vg) -sin(psi_vg); sin(psi_vg) cos(psi_vg)]*v_sg;
virtgoal = [v_vg(2)+cur_x v_vg(1)+cur_y];
 end