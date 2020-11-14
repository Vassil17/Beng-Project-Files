function [x,y,theta]=drawSensorCone(cur_psi,cur_x,cur_y,draw)
R=1; % radius of 1m

psi = -cur_psi+pi/2;
% cone width is 15 degrees, sofrom -7.5 to +7.5 degrees
start = psi - 0.1309; 
ending = psi + 0.1309;
theta=linspace(start,ending,R/0.01);
x = R*cos(theta)+cur_y;
y=R*sin(theta)+cur_x;
x=[x cur_y x(1)];
y=[y cur_x y(1)];
if draw == 1, h = patch(x,y,'b','FaceAlpha',0.1); end

end