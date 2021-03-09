function [psi_sg,psi_vg,subgoal,virtgoal,gap,closestGap]=TGF_algorithm_EG(obstacleMap,...
    scan,cur_x,cur_y,angleToGoal,goal,cur_psi,angleT2)
%
% This function contains the modified Tangential gap flow algorithm (TGF*)
% that is used by the Escape Gap algorithm. It searches for gaps between
% obstacles within the sensing radius of the robot and selects the gap
% closest to the pre-determined by the T2 algorithm angular region.
%
%
% Angles in this algorithm are defined with 0 at the current heading
% (cur_psi) and positive for (0,pi] (ccw) and negative for (0,-pi) (cw);
%

%
% LiDAR Angles are defined the same way in lidarSensor_TGF;

% First check continuity of detected obstacles in order to find gaps
i=1;
gap = struct;
terminate = 0;
% R is robot radius and Ds is security zone for obstacle avoidance
R = 0.1;
Ds = R+0.1;
counter = 0;
% Angle to the goal should be relative to current heading:
relativeAngleToGoal = cur_psi - angleToGoal;
% Define three positions (top, middle and bottom of robot) to be used for
% checking leaving condition
pos = [cur_x-0.1 cur_y; cur_x cur_y; cur_x+0.1 cur_y];
while ~terminate
    %
    % The leaving condition checks each position and its direct path to the
    % goal. The condition is only satisfied when all three paths are clear
    for ii=1:1:3
        start = pos(ii,:);
        [directPath(:,1),directPath(:,2)]=straightLine(start,goal,50);
        directPath = [directPath(:,2) directPath(:,1)];
     if ~any(checkOccupancy(obstacleMap,directPath))
         % if the path is clear for this position, add to counter
         counter=counter+1;
     end
    end
    % only when all three positions have a clear path satisfy the condition
    if counter == 3
         psi_sg = 0;
         psi_vg = 0;
         closestGap = 0;
         terminate = 1;
         continue;
    end
%     
        
% forward search:
    for index=1:1:length(scan.Angles)
        if index == length(scan.Angles)
             dist1 = scan.Cartesian(index,1) - scan.Cartesian(1,1);
             dist2 = scan.Cartesian(index,2) - scan.Cartesian(1,2);
             gapAngle = abs(scan.Angles(index) - scan.Angles(1));
        else
           dist1 = scan.Cartesian(index,1) - scan.Cartesian(index+1,1);
           dist2 = scan.Cartesian(index,2) - scan.Cartesian(index+1,2);
           gapAngle = abs(scan.Angles(index) - scan.Angles(index+1));
        end
        if abs(gapAngle) > pi
           gapAngle = 2*pi - gapAngle;  
        end
        if gapAngle >= 0.1 || sqrt(dist1^2+dist2^2)>=0.5
            gap_start(i,:) = [scan.Cartesian(index,1) -scan.Cartesian(index,2)]...
                + [cur_x cur_y];
            distance_gap = sqrt((scan.Cartesian(:,1) - scan.Cartesian(index,1)).^2 ...
                +(scan.Cartesian(:,2) - scan.Cartesian(index,2)).^2);
            angle_travelled = abs(scan.Angles(:,1) - scan.Angles(index));
            angle_travelled(index) = Inf;
            % define a constraint for the angular distance travelled to be
            % less than pi
            constraint = find(angle_travelled>=pi & angle_travelled~=angle_travelled(1));
            % Remove all entries that don't satisfy the constraint or whos
            % indices are smaller than index+1
            distance_gap(constraint) = Inf;
            distance_gap(1:index-1) = Inf;
            % For the edge discontinuity look to the closest point that
            % satisfies the conditions and that is the gap
            if ~all(distance_gap == Inf)
                j = find(distance_gap == min(distance_gap));
                gap_end(i,:) = [scan.Cartesian(j,1) -scan.Cartesian(j,2)]...
                    + [cur_x cur_y];
                % gapOrientation stores whether the gap was found with
                % forward (1) or backward(0) search.
                gapOrientation(i) = 1;
                i=i+1; 
            else
                 gap_start(i,:) = [];
            end
        end
    end
    
% backward search
    for index=length(scan.Angles):-1:1
        if index == 1
             dist1 = scan.Cartesian(index,1) - scan.Cartesian(length(scan.Angles),1);
             dist2 = scan.Cartesian(index,2) - scan.Cartesian(length(scan.Angles),2);
             gapAngle = abs(scan.Angles(index) - scan.Angles(length(scan.Angles)));
        else
           dist1 = scan.Cartesian(index,1) - scan.Cartesian(index-1,1);
           dist2 = scan.Cartesian(index,2) - scan.Cartesian(index-1,2);
           gapAngle = abs(scan.Angles(index) - scan.Angles(index-1));
        end
        if abs(gapAngle) > pi
           gapAngle = 2*pi - gapAngle;  
        end
        if gapAngle >= 0.1 || sqrt(dist1^2+dist2^2)>=0.5
            gap_start(i,:) = [scan.Cartesian(index,1) -scan.Cartesian(index,2)]...
                + [cur_x cur_y];
            distance_gap = sqrt((scan.Cartesian(:,1) - scan.Cartesian(index,1)).^2 ...
                +(scan.Cartesian(:,2) - scan.Cartesian(index,2)).^2);
            angle_travelled = abs(scan.Angles(index) - scan.Angles(:,1));
            angle_travelled(index) = Inf;
            % define a constraint for the angular distance travelled to be
            % less than pi
            constraint = find(angle_travelled>=pi & angle_travelled~=angle_travelled(end));
            % Remove all entries that don't satisfy the constraint or whos
            % indices are smaller than index+1
            distance_gap(constraint) = Inf;
            distance_gap(index:(length(scan.Angles)-1),1) = Inf;
            if ~all(distance_gap == Inf)
                j = find(distance_gap == min(distance_gap));
                gap_end(i,:) = [scan.Cartesian(j,1) -scan.Cartesian(j,2)]...
                    + [cur_x cur_y];
                gapOrientation(i) = 0;
                i=i+1; 
            else
                 gap_start(i,:) = [];
            end
        end
    end
    
    if  i==1 % i.e. if there are no gaps
        psi_sg = 0;
        closestGap = 0;
        noGaps = 1;
    else
        for gapNumber = 1:i-1
            
            
           y=[gap_start(gapNumber,1) gap_end(gapNumber,1)];
           x=[gap_start(gapNumber,2) gap_end(gapNumber,2)];
           start=[x(1),y(1)];
           ending = [x(2),y(2)];
           pts = 50;
           [x_i,y_i]=straightLine(start,ending,pts);
 
           gap(gapNumber).Coordinates = [x_i' y_i'];
           gap(gapNumber).Width = sqrt((x_i(1)-x_i(end))^2 + (y_i(1)-y_i(end))^2);


           % Find the smallest angle to each gap (either to its beginning or end)
           % And then calculate the Angle between each gap and the goal line.
           gap(gapNumber).Centre = [x_i(ceil(end/2)) y_i(ceil(end/2))];
           % the left and right side orientation depends on which search
           % found the gap(1 is forward, 0 is backward search)
           if gapOrientation(gapNumber) == 1
               [~,SideAngle(1)] = los_auto(cur_x,cur_y,[y_i(1) x_i(1)]);
               [~,SideAngle(2)] = los_auto(cur_x,cur_y,[y_i(end) x_i(end)]);
               SideAngle(1) = cur_psi - SideAngle(1);
               SideAngle(2) = cur_psi - SideAngle(2);
               gap(gapNumber).Sides(1,:) = [x_i(1) y_i(1) SideAngle(1)];
               gap(gapNumber).Sides(2,:) =[x_i(end) y_i(end) SideAngle(2)];
           else              
               [~,SideAngle(1)] = los_auto(cur_x,cur_y,[y_i(end) x_i(end)]);
               [~,SideAngle(2)] = los_auto(cur_x,cur_y,[y_i(1) x_i(1)]);
               SideAngle(1) = cur_psi - SideAngle(1);
               SideAngle(2) = cur_psi - SideAngle(2);
               gap(gapNumber).Sides(1,:) = [x_i(end) y_i(end) SideAngle(1)];
               gap(gapNumber).Sides(2,:) =[x_i(1) y_i(1) SideAngle(2)];
           end
           % Need angles to be relative to the current heading and in the algorithm
           % notation (clockwise from heading is negative)
          
           % Sides stores the right side (row 1) and the left side (row 2) of gap

            index = find(abs(gap(gapNumber).Sides(:,3))==min(abs(SideAngle(1)),abs(SideAngle(2))));
            gap(gapNumber).ClosestGapAngle = gap(gapNumber).Sides(index,3);
            % For EG mindist is defined with the angle of Rt rather than
            % angle to goal
            gap(gapNumber).mindist = min(abs((cur_psi - angleT2) - gap(gapNumber).Sides(:,3)));
%            gap(gapNumber).AngleDifference = abs((cur_psi - angleToGoal) - gap(gapNumber).ClosestGapAngle);

           gapSpan(gapNumber,:) = [SideAngle(1) SideAngle(2)];
        end
        % Filter through the gapSpan to find which gap falls within another
        for k = 1:1:gapNumber
%          filterIndex = find(abs(gapSpan(k,1))>=abs(gapSpan(:,1)) & abs(gapSpan(k,2))>=abs(gapSpan(:,2)));
%          filterIndex = filterIndex';
%          clear coordinate;
%          for id = 1:length(filterIndex)
%           coordinate(id,:) = gap(filterIndex(id)).Centre;
%          end
%         distance = sqrt((cur_x - coordinate(:,2)).^2 + (cur_y - coordinate(:,1)).^2);
% 
%         mainGap = find(distance == min(distance));
%         if k ~= mainGap
%            gap(k).Gap =0;
%            continue;
%         else
           if gap(k).Width < 0.5
               % 0 means gap is blocked
              gap(k).Gap =0;
              continue;
           else
               % 1 means gap is allowed
               gap(k).Gap = 1;                  
           end
%         end
        end
        % Delete gaps which are inaccessible
        idx = find([gap.Gap] == 0);
        gap(idx) = [];
       
        
        %If there are no gaps, head straight to goal
        if ~all([gap.Gap]==0)
           noGaps = 0;
          closestGap  = find([gap.mindist] == min([gap.mindist]));
          %in case there are multiple gaps with the same angle difference,
          %choose the first one
          closestGap = min(closestGap);
        else
          noGaps=1;
        end
      if noGaps == 0
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
      end
    end
         indx = find(scan.Ranges == min(scan.Ranges));
         dist = [scan.Cartesian(indx,1) -scan.Cartesian(indx,2)]+[cur_x cur_y];
         [~,globalBeta] = los_auto(cur_x,cur_y,[dist(1) dist(2)]);
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
    terminate = 1;
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
%     angleT2 = angleT2 - goalAngle;
%     v_goalT2 = [cos(angleT2) -sin(angleT2); sin(angleT2) cos(angleT2)]*v_goal;
%     goalT2 = [v_goalT2(2)+cur_x v_goalT2(1)+cur_y];
%     % Rotate v_goal by psi_sg to obtain v_sg (subgoal)
%     % But psi_sg is defined relative to current heading so convert:
    psi_sg = cur_psi + psi_sg;
    v_sg = [cos(psi_sg) -sin(psi_sg); sin(psi_sg) cos(psi_sg)]*v_goal;
    subgoal = [v_sg(2)+cur_x v_sg(1)+cur_y];
    % Then analogous with the virtual goal
    v_vg = [cos(psi_vg) -sin(psi_vg); sin(psi_vg) cos(psi_vg)]*v_sg;
    virtgoal = [v_vg(2)+cur_x v_vg(1)+cur_y];

 end