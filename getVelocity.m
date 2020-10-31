function [desired_vel] = getVelocity(desired_coord,cur_x,cur_y)

    distance = pdist([desired_coord;cur_x,cur_y],'euclidean');    
    desired_vel = 1;
  if abs(distance) < 0.3
      desired_vel = 0;
  end

end