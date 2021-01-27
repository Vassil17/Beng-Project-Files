function []=drawSectors(angle,cur_x,cur_y,sector,K,i,Rright,Rleft,tenacity)
R=2; % radius of 2m
%angle = angle+pi/2;
% cone width is 10 degrees, but for two sensors so from -10 to +10 degrees
start = angle; 
ending = angle +  2*pi/K;
theta=linspace(start,ending,R/0.01);
x = R*cos(theta)+cur_y;
y=R*sin(theta)+cur_x;
x=[x cur_y x(1)];
y=[y cur_x y(1)];
% 0 is left tenacity, 1 is right tenacity
if i == Rleft && tenacity == 0
    h = patch(x,y,'g','FaceAlpha',0.5);
elseif i==Rright && tenacity == 1
    h = patch(x,y,'g','FaceAlpha',0.5);
elseif strcmp(sector,'blocked')
    h = patch(x,y,'r','FaceAlpha',0.5);
else
    h = patch(x,y,'w','FaceAlpha',0.5);
end
end