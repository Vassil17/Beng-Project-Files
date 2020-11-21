function [desired_vel] = getVelocity(desired_coord,cur_x,cur_y)

    distance = pdist([desired_coord;cur_x,cur_y],'euclidean');    
    
  if abs(distance) < 0.4
      desired_vel = 0.1;
  elseif abs(distance) < 0.2
      desired_vel = 0;
  else
      desired_vel = 0.75;
  end

end