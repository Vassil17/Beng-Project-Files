% First check continuity of detected obstacles in order to find gaps
i=1;
gap = struct;
for index=1:1:(length(scan.Angles)-1)
    if abs(scan.Angles(index) - scan.Angles(index+1)) >= 0.1
        gap_start(i,:) = scan.Cartesian(index,:) + [cur_y cur_x];
        gap_end(i,:) = scan.Cartesian(index+1,:) + [cur_y cur_x];
        i=i+1;
    end
end
for gapNumber = 1:i-1
   
   x=[gap_start(gapNumber,1) gap_end(gapNumber,1)];
   y=[gap_start(gapNumber,2) gap_end(gapNumber,2)];
   diff = x(2) - x(1);
   x_i = x(1):sign(diff)*0.1:x(2);
   y_i = interp1(x,y,x_i,'linear');
   plot(x_i,y_i);
   gap(gapNumber).Coordinates = [x_i' y_i'];
   gap(gapNumber).Width = sqrt((x_i(1)-x_i(end))^2 + (y_i(1)-y_i(end))^2);
   if gap(gapNumber).Width < 0.5
      gap(gapNumber).Gap = 'blocked';
   else
       gap(gapNumber).Gap = 'allowed';
   end

   % Find the smallest angle to each gap (either to its beginning or end)
   % And then calculate the Angle between each gap and the goal line.
   gap(gapNumber).Centre = [x_i(ceil(end/2)) y_i(ceil(end/2))];
   [~,angleToGoal] = los_auto(cur_x,cur_y,goal);
   [~,angleToGap1] = los_auto(cur_x,cur_y,[x_i(1) y_i(1)]);
   [~,angleToGap2] = los_auto(cur_x,cur_y,[x_i(end) y_i(end)]);
   angleToGap = min(angleToGap1,angleToGap2);
   gap(gapNumber).Angle = abs(angleToGoal - angleToGap);
end
% Find which gap has the smallest angle to the goal
closestGap = find(min(gap.Angle));