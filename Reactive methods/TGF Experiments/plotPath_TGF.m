clear;
clc;
load('TGF_scenario9_1.mat');
figure(1);
clf;
show(obstacleMap);grid on; hold on;
xlabel('X position [metres]');
ylabel('Y position [metres]');
title('Map of the environment');
 % plot the goal point
goalPlot(1) = plot(start(2),start(1),'Marker','x','MarkerEdgeColor','red',...
    'LineWidth',2,'MarkerSize',12,'DisplayName','Start');
goalPlot(2) = plot(goal(2),goal(1),'Marker','x','MarkerEdgeColor','[0.9290, 0.6940, 0.1250]',...
    'LineWidth',2,'MarkerSize',12,'DisplayName','Goal');
set(goalPlot,'linestyle','none');
 %   drawrobot(0.2,xi(20)+5,xi(19)+5,xi(24),'b');
trajectory(1) = plot(xio(:,20)+5,xio(:,19)+5,'blue','LineStyle',...
    '--','LineWidth',2,'DisplayName','Path');
trajectory(1).Color(4) = 0.75;
legend('FontSize',9);