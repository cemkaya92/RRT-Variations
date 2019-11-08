% This file is created by U. Cem Kaya - 2018
function obstacles = polygonalObstacles(mapSize,figHandle)

th = linspace(0,2*pi,6);
xv{1} = 0.05*cos(th);
yv{1} = 0.05*sin(th);

th = linspace(0,2*pi);
xv{2} = 0.025*cos(th);
yv{2} = 0.05*sin(th);

xv{3} = [0 0.18 0.18 0 0];
yv{3} = [0 0   0.015 0.015 0];

xv{4} = [0 0.035 -0.035 0 ];
yv{4} = [0 0.07  0.054 0];

xv{5} = [0 0.13 0.13 0 0];
yv{5} = [0 0.008 0.036 0.025 0];

Offset = [0.15 0.15; 0.025 0.2; 0 0.305; 0.23 0.14; 0.165 0.22];
% %Define a rotation matrix
% R1 = [ cos(phi) sin(phi); -sin(phi) cos(phi) ]';
for i = 1:length(Offset)
    
    r = [xv{i} ; yv{i}] + [mapSize(1,1) + Offset(i,1); mapSize(1,2)+Offset(i,2)]; % in km
    
    x0 = sum(r(1,1:end-1))/length(r(1,1:end-1));
    y0 = sum(r(2,1:end-1))/length(r(2,1:end-1));
    outerRadius = max(sqrt((r(1,1:end-1)'-x0).^2+(r(2,1:end-1)'-y0).^2));
    
    obstacles(i).pos = r;
    obstacles(i).center = [x0;y0];
    obstacles(i).outerRadius = outerRadius;
    % plot polygon
    
%     th = linspace(0,2*pi);
    x_cir = x0 + outerRadius*cos(th);
    y_cir = y0 + outerRadius*sin(th);
    
    figure(figHandle)
    h = plot(r(1,:)',r(2,:)','k-','linewidth',2,'DisplayName','No-Fly-Over Zones');
    hold on
% %     plot(x_cir,y_cir,'b','linewidth',2);
% %     hold on
% %     plot(x0,y0,'ro');
% %     hold on

end


end