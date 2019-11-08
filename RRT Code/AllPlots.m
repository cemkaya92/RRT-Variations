% This file is created by U. Cem Kaya - 2018
function h = AllPlots(figHandle,MultiplePaths,finalPath,PREM,X,Y,mapSize,BuildingCenters,corners_X,corners_Y,start,finish)

PREM_value = PREM.layer1;

xrange = [mapSize(1,1) mapSize(2,1)];
yrange = [mapSize(1,2) mapSize(2,2)];
% PREM_value = -PREM_value;
%%% linke bak https://www.mathworks.com/help/map/ref/contour3m.html
figure(figHandle)

ax = gca;               % get the current axis
ax.Clipping = 'off';    % turn clipping off

img = imread('map1','png');
 
% set the range of the axes
% The image will be stretched to this.
% min_x = 0;
% max_x = 8;
% min_y = 0;
% max_y = 6;
% image('CData',img,'XData',[4 8],'YData',[-1/4 1/4]);

% colormap(gray);

% Flip the image upside down before showing it
% imagesc([min_x max_x], [min_y max_y], flipud(img));
imagesc(xrange, yrange, flipud(img));

% NOTE: if your image is RGB, you should use flipdim(img, 1) instead of flipud.
 
hold on;
% patch([corners_X(:,1)'; corners_X(:,2)'; corners_X(:,3)'; corners_X(:,4)'],[corners_Y(:,1)'; corners_Y(:,4)'; corners_Y(:,2)'; corners_Y(:,3)'],'w');
hold on
% plot(BuildingCenters(:,1),BuildingCenters(:,2),'*','color','c','MarkerSize',5);
% hold on

% set the y-axis back to normal.
set(gca,'ydir','normal');

h = surf(X,Y,PREM_value,'FaceAlpha','flat','AlphaDataMapping','scaled','AlphaData',log(PREM_value));
% [x,y] = meshgrid([-2:.2:2]);
% z = x.*exp(-x.^2-y.^2);
% surf(x,y,z+.001,'FaceAlpha','flat',...
% 'AlphaDataMapping','scaled',...
% 'AlphaData',gradient(z),...
% 'FaceColor','blue');
% h = contourf(X,Y,PREM_value);
% alpha 0.5
caxis([min(PREM_value(:))-.5*range(PREM_value(:)),max(PREM_value(:))]);
set(h, 'EdgeColor', 'none');
set(h, 'LineStyle', '-');
set(h, 'LineWidth', 0.001);
set(h, 'EdgeAlpha',0.7);
set(h, 'FaceLighting','gouraud');
% % set(h, 'EdgeColor', 'k');
% % set(h, 'LineStyle', '-');
% % set(h, 'LineWidth', 0.01);
% % set(h, 'EdgeAlpha',0.8);
% % set(h, 'FaceLighting','gouraud');
% set(h, 'EdgeLighting','gouraud');
% light
% lighting gouraud 
material dull
alpha(h,'z')
% alpha(h,0.6)
% A.EdgeColor = 'r';
axis([xrange yrange])

grid off;
% view(3);
% view(0,90);
title('\bf RRT Generated Path - Top View')
xlabel(' X, km')
ylabel(' Y, km')
% xlabel(' Longitude, km')
% ylabel(' Latitude, km')
zlabel(' PREM Value ')

% colormap parula
colormap jet
%  V=max(max(PREM_value))*[0:.04:1].^3;
 Vmax=max(max(PREM_value));
% V=[linspace(1e-16,1e-1,100)];
% % V =[];
% % for jj = 1:10
% %     V = [V Vmax];
% %     Vmax = V(end)/2;
% % end
% figure(figHandle)
% contour(X,Y,PREM_value,V,'Linewidth',1.5)
% axis([xrange yrange])
% axis equal
% grid on;
% title('\bf Probabilistic Risk Exposure Map 2-D')
% xlabel(' Longitude, km')
% ylabel(' Latitude, km')
% xtickangle(45)
hold on


%             % Plot backtrack in red line
% for jj = 1:length(MultiplePaths)
%     
%     plot([MultiplePaths{1,jj}(:).x]',[MultiplePaths{1,jj}(:).y]','LineWidth', 2);
%     hold on
% end


m = plot3(start(1) , start(2),Vmax,'g^','LineWidth',3,'MarkerEdgeColor','g','MarkerSize',8);
 hold on      
n = plot3(finish(1), finish(2),Vmax,'rs','LineWidth',4,'MarkerEdgeColor','r','MarkerSize',10);
 hold on 
% First Path red line   
k =  plot3([MultiplePaths{1,1}(:).x]',[MultiplePaths{1,1}(:).y]',Vmax*ones(length(MultiplePaths{1,1}),1),'r-','LineWidth', 2);
    hold on
% Final Path blue line   
l =  plot3([finalPath(:).x]',[finalPath(:).y]',Vmax*ones(length(finalPath),1),'b-.','LineWidth', 3);
    hold on

% % o =  plot3([FinalSmoothedPath(:).x]',[FinalSmoothedPath(:).y]',Vmax*ones(length(FinalSmoothedPath),1),'c','LineWidth', 1.5);
% %     hold on
% l =  plot3([MultiplePaths{1,end}(:).x]',[MultiplePaths{1,end}(:).y]',Vmax*ones(length(MultiplePaths{1,end}),1),'b-','LineWidth', 3);
%     hold on
% k =  plot3([finalPath(:).x]',[finalPath(:).y]' ,Vmax*ones(length(finalPath),1), 'c', 'LineWidth', 3);
% k =  plot3([finalPath(:).x]',[finalPath(:).y]' ,[finalPath(:).CumulativeCost]', 'g', 'LineWidth', 3);
%   hold on;
  
lgd = legend([k l m n],'First Path','Final Path','start','finish','location','north','orientation','horizontal');
% % lgd = legend([k l o m n],'First Path','Final Path','Smoothed Final Path','start','finish','location','north','orientation','horizontal');
% % lgd = legend([l m n],'Direct Path','start','finish','location','northwest');

lgd.FontSize = 8;
lgd.Color = 'w';
lgd.Position = [0.35 0.875 0.35 0.04];

cb1 = colorbar;
cb1.Limits = [0 Vmax];
cb1.Label.String = 'PREM value';
cb1.Location = 'south';
cb1.Position = [0.15 0.125 0.35 0.02];
cb1.AxisLocation = 'in';


end
