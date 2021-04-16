function []=drawSectors(angle,cur_x,cur_y,sector,K,i,RGoal,range)
start = angle; 
ending = angle +  2*pi/K;
theta=linspace(start,ending,range/0.01);
x = range*cos(theta)+cur_y;
y=range*sin(theta)+cur_x;
x=[x cur_y x(1)];
y=[y cur_x y(1)];

if i==RGoal && strcmp(sector,'allowed')
    h = patch(x,y,'g','FaceAlpha',0.25,'HandleVisibility','off');
elseif strcmp(sector,'blocked')
    h = patch(x,y,'r','FaceAlpha',0.25,'HandleVisibility','off');
else
    h = patch(x,y,'w','FaceAlpha',0.25,'HandleVisibility','off');
end
end