clear;
clc;
load("TGF_scenario10.mat");
figure(1);
clf;
show(obstacleMap);grid on; hold on;
drawnow;
xlabel('X position [metres]');
ylabel('Y position [metres]');
title('Map of the environment');
 % plot the goal point
goalPlot(1) = plot(start(2),start(1),'Marker','x','MarkerEdgeColor','red',...
    'LineWidth',2,'MarkerSize',12,'DisplayName','Start');
goalPlot(2) = plot(goal(2),goal(1),'Marker','x','MarkerEdgeColor','[0.9290, 0.6940, 0.1250]',...
    'LineWidth',2,'MarkerSize',12,'DisplayName','Goal');
set(goalPlot,'linestyle','none');
trajectory(1) = plot(xio(:,20)+5,xio(:,19)+5,'blue','LineStyle',...
    '--','LineWidth',2,'DisplayName','Path with 10m range');
trajectory(1).Color(4) = 0.75;
%     for i=1:1:K
%         drawSectors(environment.angle(i),cur_x,cur_y,environment.sector(i),K,i,R,range)
%     end

%
%

  load("TGF_scenario10_3m.mat");
 trajectory(2) = plot(xio(:,20)+5,xio(:,19)+5,'red',...
      'LineStyle','--','LineWidth',2,'DisplayName','Path with 3m range');
  trajectory(2).Color(4) = 0.5;
legend('FontSize',9);
%
%     for i=1:1:K
%         drawSectors(environment.angle(i),cur_x,cur_y,environment.sector(i),K,i,R,range)
%     end
q_arr=[];
for i=1:length(q_arr)
    q = q_arr(i);
    %drawrobot(0.2,plotStorage(q,1),plotStorage(q,2),plotStorage(q,3),'b');
    cntr = q_arr(i);
     for k=1:K
        angle = sectorPlotStorage(cntr).env.angle(k);
        sector = sectorPlotStorage(cntr).env.sector(k);
        cur_x = sectorPlotStorage(cntr).pos(1);
        cur_y = sectorPlotStorage(cntr).pos(2);
        R = sectorPlotStorage(cntr).R;
        drawSectors(angle,cur_x,cur_y,sector,K,k,R,range)
    end
end
%
%
%exportgraphics(gcf,'T2_scenario6.jpg','Resolution',1080);
