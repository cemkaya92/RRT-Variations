% This file is created by U. Cem Kaya - 2018
function PURM_plot(PURM,dX,dY,pathFigs)

purmInd = figure;



    phi = PURM.theta;
    ReachableRadius = PURM.reachableRadius*1000;
    failureRate = PURM.failureRate;
    lambda = sum(failureRate);
    dArea = dX*1000*dY*1000;
    
X = linspace(0,4*max(ReachableRadius(:,1)),500);
Y = linspace(0,4*max(ReachableRadius(:,2)),500);  
xi = 0:10:4*max(ReachableRadius(:,1));
yi = 0:10:4*max(ReachableRadius(:,2));
[Xi,Yi] = meshgrid(xi,yi) ;
[X,Y] = meshgrid(X,Y);
    %% some closed polygon
th = linspace(0,2*pi) ;
% phi = [60*pi/180; 60*pi/180 ] ;
X0=10;
Y0=20;
a=ReachableRadius(:,1);
b=ReachableRadius(:,2);

colorCode = ['g','y','r'];
Z = zeros(size(X)) ;

for i = 1:length(phi)
        % the ellipse in x and y coordinates 
ellipse_x_r1  =  a(i)*cos( th );
ellipse_y_r1  =  b(i)*sin( th );

Zi = zeros(size(X));
%Define a rotation matrix
R1 = [ cos(phi(i)) -sin(phi(i)); sin(phi(i)) cos(phi(i)) ];


%let's rotate the ellipse to some angle phii
r_ellipse1 = [X0;Y0] + R1 *([a(i);0] + [ellipse_x_r1;ellipse_y_r1]); 


%% get points lying inside polygon
idx = inpolygon(X,Y,r_ellipse1(1,:)',r_ellipse1(2,:)') ;
Zi(idx) = (failureRate(i)/lambda)*(1/(dArea*length(Zi(idx))));
figure(purmInd)
h(i) = surf(X,Y,Zi,'FaceColor',colorCode(i),'FaceAlpha',0.7,'EdgeColor', 'none','EdgeAlpha',0.1,'FaceLighting','gouraud');
hold on

Z(idx) = Z(idx)+(failureRate(i)/lambda)*(1/(dArea*length(Z(idx)))) ;  % assign one which lie inside 

end



surf(Xi,Yi,10^(-8)*ones(size(Xi)),'FaceColor','w','FaceAlpha',1,'EdgeColor', 'k','EdgeAlpha',1,'FaceLighting','gouraud')

uas = plot3(X0,Y0,max(max(Zi)),'>','LineWidth',2,'MarkerEdgeColor','b','MarkerSize',10);
axis([0 60 0 50])
view([0 90])

title('\bf UAS Impact Domains for Different Failure Modes ');
xlabel(' X, m');
ylabel(' Y, m');
% axis([1.1*[-stepSize stepSize] 1.1*[-stepSize stepSize]]);
legend([h(:); uas],'F_1','F_2','F_3','UAS Fail Position','location','north','orientation','horizontal');

fncSaveFig(1,pathFigs,'ImpactDomain_TOP_seperate1',11,6,4);

figure
k = surf(X,Y,Z,'FaceAlpha',0.7,'EdgeColor', 'none','EdgeAlpha',0.1,'FaceLighting','gouraud');
% h(i) = fill(r_ellipse1(1,:),r_ellipse1(2,:),'-','linewidth',2.5);
% alpha(h(i),0.7)
hold on
surf(Xi,Yi,10^(-8)*ones(size(Xi)),'FaceColor','w','FaceAlpha',1,'EdgeColor', 'k','EdgeAlpha',1,'FaceLighting','gouraud')
hold on
plot3(X0,Y0,max(max(Zi)),'>','LineWidth',2,'MarkerEdgeColor','b','MarkerSize',10)
grid on

map = [0, 1, 0
    0.9, 1, 0
    1, 1, 0
    1, 0.5, 0.3
    1, 0, 0];
colormap(map)
axis([0 60 0 50])
view([0 90])

title('\bf Combined UAS Impact Domain');
xlabel(' X, m');
ylabel(' Y, m');

cb1 = colorbar;
% cb1.Limits = [0 Vmax];
cb1.Label.String = 'Density Value';
cb1.Location = 'north';
% cb1.Position = [0.15 0.125 0.35 0.02];
cb1.AxisLocation = 'in';

fncSaveFig(1,pathFigs,'ImpactDomain_TOP_combined1',11,6,4);

% axis([1.1*[-stepSize stepSize] 1.1*[-stepSize stepSize]]);

% h = surf(X,Y,PURM_total,'FaceColor','flat','FaceAlpha',0.7,'EdgeColor', 'k','EdgeAlpha',0.1,'FaceLighting','gouraud');
% material dull
% hold on
% axis([xrange yrange]);


end

