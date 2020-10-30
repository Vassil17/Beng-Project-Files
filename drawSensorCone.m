function [x,y,theta]=drawSensorCone(cur_psi,cur_x,cur_y,draw)
R=1; % radius of 1m

psi = -cur_psi+pi/2;
start = psi - pi/4;
ending = psi + pi/4;
theta=linspace(start,ending,R/0.01);
x = R*cos(theta)+cur_y;
y=R*sin(theta)+cur_x;
x=[x cur_y x(1)];
y=[y cur_x y(1)];
if draw == 1, h = patch(x,y,'b','FaceAlpha',0.1); end

end