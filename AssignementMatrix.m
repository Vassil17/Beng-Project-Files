%
% Generate Obstacle Matrix for Assignment 
%

wall = WallGeneration1(-1, 1,1.2,1.2,'h');
wall2 = WallGeneration1(-3, -3, -2, 2,'v');
wall3 = WallGeneration1(2, 2, -3, 1,'v');
wall4 = WallGeneration1(-3, -1, 4, 4,'h');

% Create Environment
max_x = 10;
max_y = 10;

Obs_Matrix = zeros(max_x/0.01,max_y/0.01);

for x=1:length(wall)
    
    xpos = (wall(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall2)
    
    xpos = (wall2(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall2(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall3)
    
    xpos = (wall3(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall3(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall4)
    
    xpos = (wall4(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall4(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
end