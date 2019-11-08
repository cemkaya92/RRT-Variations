% This file is created by U. Cem Kaya - 2018
%% Draw PTEM circle 
function [PREM_value,dX,dY,X,Y] = PREMplot(density,prem,xrange,yrange,Grid_x,Grid_y,Population)

% N_grid = 50;    % Grid resolution: Grid size will be N_grid by N_grid
dX = (xrange(2)-xrange(1))/Grid_x;
dY = (yrange(2)-yrange(1))/Grid_y;
% dArea = dX*dY;

% Create the X and Y list for contour plot
[X,Y] = meshgrid((xrange(1)+dX/2:dX:xrange(2)-dX/2),(yrange(1)+dY/2:dY:yrange(2)-dY/2)); % These X and Y's are the center of each small grid
% [X,Y] = meshgrid(xrange(1):dX:xrange(2),yrange(1):dY:yrange(2));

PREM_value = zeros(Grid_y,Grid_x);  % Initialize the PREM evaluation
% PREM_value = zeros(length(yrange(1):dY:yrange(2)),length(xrange(1):dX:xrange(2)));  % Initialize the PREM evaluation

for ii = 1:length(prem.pdfs)    % For each PDF in the PREM
    
    m1 = prem.pdfs(ii).m(1);
    m2 = prem.pdfs(ii).m(2);
    s1 = sqrt(prem.pdfs(ii).k(1,1));
    s2 = sqrt(prem.pdfs(ii).k(2,2));
    
    % Evaluate and superimpose each PDF
    PREM_value = PREM_value + density(ii)*exp(-1/2*(((X - m1)/s1).^2 + ((Y - m2)/s2).^2))/(2*pi*s1*s2);
    
end
% PREM_value = PREM_value*Population;
%  V=max(max(PREM_value))*[0:.04:1].^3;
 Vmax=max(max(PREM_value));
% V=[linspace(1e-16,1e-1,100)];
V =[];
for jj = 1:10
    V = [V Vmax];
    Vmax = V(end)/2;
end
pathFigs = 'C:\Users\Cem.CEMPC\Desktop\CEM PREM modified GIS\figs\';
% figure(figureHandle)

% % % figure(1)
% % % h = surf(X,Y,PREM_value);
% % % % alpha 0.5
% % % caxis([min(PREM_value(:))-.5*range(PREM_value(:)),max(PREM_value(:))]);
% % % set(h, 'EdgeColor', 'k');
% % % set(h, 'LineStyle', '-');
% % % set(h, 'LineWidth', 0.01);
% % % set(h, 'EdgeAlpha',0.8);
% % % set(h, 'FaceLighting','gouraud');
% % % % set(h, 'EdgeLighting','gouraud');
% % % % light
% % % % lighting gouraud 
% % % material dull
% % % alpha 0.4
% % % % A.EdgeColor = 'r';
% % % axis([xrange yrange])
% % % 
% % % grid on;
% % % % view(3);
% % % view(0,90);
% % % title('\bf Probabilistic Risk Exposure Map 3-D')
% % % xlabel(' Longitude, km')
% % % ylabel(' Latitude, km')
% % % zlabel(' PREM Value ')

% fncSaveFig(1,pathFigs,'PREM_3D',11,8,6) 

% figure(2)
% contour(X,Y,PREM_value,V)
% axis([xrange yrange])
% axis equal
% grid on;
% title('\bf Probabilistic Risk Exposure Map 2-D')
% xlabel(' Longitude, km')
% ylabel(' Latitude, km')
% xtickangle(45)

% fncSaveFig(1,pathFigs,'PREM_2D',11,6,6) 

% contour(X,Y,PREM_value)
% h = surfc(X,Y,PREM_value);
% set(h,'LineStyle','none')
% set(h,'edgecolor','none')
% caxis([max(max(PREM_value))/2 max(max(PREM_value))])
% colormap hsv
% colorbar
% hold on
%contour(X,Y,PREM_value,[prem.threshold prem.threshold],'k-','LineWidth',3)

end