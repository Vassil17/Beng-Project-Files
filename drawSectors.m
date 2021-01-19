function []=drawSectors(angle,cur_x,cur_y,blocked)
R=3; % radius of 1m
angle = angle+pi/2;
% cone width is 10 degrees, but for two sensors so from -10 to +10 degrees
start = angle; 
ending = angle +  0.1745;
theta=linspace(start,ending,R/0.01);
x = R*cos(theta)+cur_y;
y=R*sin(theta)+cur_x;
x=[x cur_y x(1)];
y=[y cur_x y(1)];
if blocked == 1
    h = patch(x,y,'r','FaceAlpha',0.5);
else
    h = patch(x,y,'w','FaceAlpha',0.5);
end
end