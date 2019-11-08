% This file is created by U. Cem Kaya - 2018
function [riskExposureAlongPath,PositionRisk,tr_test,T] = RiskExposureSegment(pathTree,PREM,extended,...
    PathNodeID,xrange,yrange,dX,dY,PURM,X,Y,Population,T)


    x1 = extended(1);  y1 = extended(2);
    x2 = pathTree(PathNodeID).x;  y2 = pathTree(PathNodeID).y; 
    length_segment = Dist([x1 y1]',[x2 y2]');
    
    Risk0 = pathTree(PathNodeID).cost;
    privacy_risk0 = pathTree(PathNodeID).privacyCost;
    
    Angles = PURM.theta;
    ReachableRadius = PURM.reachableRadius;
    failureRate = PURM.failureRate;
    lambda = sum(failureRate);
    dArea = dX*dY;
    
    %% some closed polygon
th = linspace(0,2*pi,10) ;
% phi = [atan2((y1-y2),(x1-x2)); pathTree(PathNodeID).psi ] ;
phi = atan2((y1-y2),(x1-x2))  ;
% phi = [60*pi/180; 60*pi/180 ] ;
% X0=[x1;x2];
% Y0=[y1;y2];
X0=x1;
Y0=y1;
a=ReachableRadius(:,1);
b=ReachableRadius(:,2);

% %         % the ellipse in x and y coordinates 
% % ellipse_x_r1  =  a(1)*cos( th );
% % ellipse_y_r1  =  b(1)*sin( th );
% % 
% % 
% % %Define a rotation matrix
% % R1 = [ cos(phi) sin(phi); -sin(phi) cos(phi) ]';
% % % R2 = [ cos(phi(2)) sin(phi(2)); -sin(phi(2)) cos(phi(2)) ]';
% % 
% % %let's rotate the ellipse to some angle phii
% % r_ellipse1 = R1 * [ellipse_x_r1;ellipse_y_r1]+[X0;Y0]; %r_ellipse1 = R1 * [ellipse_x_r1-X0(1);ellipse_y_r1-Y0(1)]+[X0(1);Y0(1)];
% % % r_ellipse2 = R2 * [ellipse_x_r2;ellipse_y_r2]+[X0(2);Y0(2)];  
% % 
% % 
% % %% get points lying inside polygon
% % idx1 = inpolygon(X,Y,r_ellipse1(1,:)',r_ellipse1(2,:)') ;
% % Z1 = zeros(size(X)) ;
% % Z1(idx1) = 1/(dArea*length(Z1(idx1))) ;  % assign one which lie inside 
% % % figure
% % % plot(r_ellipse1(1,:),r_ellipse1(2,:),'-x')
% % % surf(X,Y,Z1);
% % % % idx2 = inpolygon(X,Y,r_ellipse2(1,:)',r_ellipse2(2,:)') ;
% % % % Z2 = zeros(size(X)) ;
% % % % Z2(idx2) = 1/(dArea*length(Z2(idx2))) ;  % assign one which lie inside 
% % % figure
% % % plot(r_ellipse2(1,:),r_ellipse2(2,:),'-x')
% % % surf(X,Y,Z2);
% % 
% %         [rows1,columns1] = find( Z1 ~= 0); 
% %         
% % %         [rows2,columns2] = find( Z2 ~= 0); 

x_small = x2-max(a):dX:x2+max(a);
y_small = y2-max(a):dY:y2+max(a);
[X_small,Y_small] = meshgrid(x_small,y_small);

ImpactDist_small = zeros(size(X_small));

 for j = 1:length(a)
        
     Theta = phi+Angles(j);
%  Theta = -40*pi/180;    
     sigma_x = a(j)/3; sigma_y = b(j)/3;
% theta = 0*pi/180; % rotates clockwise

a2 = ((cos(-Theta)^2)/(2*sigma_x^2))+((sin(-Theta)^2)/(2*sigma_y^2));
b2 = (-(sin(2*(-Theta)))/(4*sigma_x^2))+((sin(2*(-Theta)))/(4*sigma_y^2));
c2 = ((sin(-Theta)^2)/(2*sigma_x^2))+((cos(-Theta)^2)/(2*sigma_y^2));


%Define a rotation matrix
R1 = [ cos(Theta) -sin(Theta); sin(Theta) cos(Theta) ]; % angle is negative because prem and purm is flipped upside down.
% upper side of
% the map
% corresponds to
% bottom of the
% matrix

% % mu = [x2;y2]+R1*[a(j);0];
mu = [x2;y2]+R1*[0.4*a(j);0];
% Sigma = [a2 b2; b2 c2];

[~,x_mu] = min((X(1,:)-mu(1)).^2);
[~,y_mu] = min((Y(:,1)-mu(2)).^2);

A = 1;
F = A*exp(-(a2*(X_small - mu(1)).^2 + 2*b2*(X_small - mu(1)).*(Y_small - mu(2)) + c2*(Y_small - mu(2)).^2));

% % figure(16)
% % plot3(x2,y2,0.1,'r>','MarkerSize',5);hold on;
% % surf(X_small,Y_small,F);
% % caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
% % view([0 90]);

edge = A*exp(-(a2*(a(j)*cos(Theta)).^2));

idx =  F > edge;

% F = F(idx)/sum(sum(F(idx))); normalized to make the sum 1
ImpactDist_small(idx) = ImpactDist_small(idx) + (failureRate(j)/lambda)*F(idx)/sum(sum(F(idx)));
    
    
 end
 
ImpactDist = zeros(size(X));
 
[rowDim,colmDim] = size(ImpactDist_small);
half_x = round(colmDim/2);
half_y = round(rowDim/2);
length_X = length(ImpactDist(1,:));
length_Y = length(ImpactDist(:,1));
if (x_mu > half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y <= length_Y)
    
    
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = upperLeft_x + colmDim-1;
    bottomRight_y = upperLeft_y + rowDim-1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small;

elseif (x_mu <= half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = 1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x =  upperLeft_x + colmDim + x_mu - half_x-1;
    bottomRight_y =  upperLeft_y + rowDim -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(:,(half_x-x_mu+1):end);

elseif (x_mu > half_x ) && (x_mu + half_x > length_X) && (y_mu > half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = length_X;
    bottomRight_y = upperLeft_y + rowDim-1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(:,1:(colmDim-((colmDim-half_x)-(length_X - x_mu))));

elseif (x_mu > half_x ) && (x_mu + half_x <= length_X) && (y_mu <= half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = 1;
    bottomRight_x =  upperLeft_x + colmDim -1;
    bottomRight_y =  upperLeft_y + rowDim +y_mu - half_y -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small((half_y-y_mu+1):end,:);

elseif (x_mu > half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y > length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = upperLeft_x + colmDim-1;
    bottomRight_y = length_Y;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(1:(rowDim-((rowDim-half_y)-(length_Y - y_mu))),:);
    
elseif (x_mu <= half_x ) && (x_mu + half_x <= length_X) && (y_mu <= half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = 1;
    upperLeft_y = 1;
    bottomRight_x =  upperLeft_x + colmDim + x_mu - half_x-1;
    bottomRight_y =  upperLeft_y + rowDim + y_mu - half_y -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small((half_y-y_mu+1):end,(half_x-x_mu+1):end);


elseif (x_mu > half_x ) && (x_mu + half_x > length_X) && (y_mu > half_y ) && (y_mu + half_y > length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = length_X;
    bottomRight_y = length_Y;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(1:(rowDim-((rowDim-half_y)-(length_Y - y_mu))),1:(colmDim-((colmDim-half_x)-(length_X - x_mu))));

elseif (x_mu > half_x ) && (x_mu + half_x > length_X) && (y_mu <= half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = 1;
    bottomRight_x =  length_X;
    bottomRight_y =  upperLeft_y + rowDim +y_mu - half_y -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small((half_y-y_mu+1):end,1:(colmDim-((colmDim-half_x)-(length_X - x_mu))));

elseif (x_mu <= half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y > length_Y)
    upperLeft_x = 1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x =  upperLeft_x + colmDim +x_mu - half_x -1;
    bottomRight_y =  length_Y;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(1:(rowDim-((rowDim-half_y)-(length_Y - y_mu))),(half_x-x_mu+1):end);

end

% %  figure(17)
% % plot3(x2,y2,0.1,'r>','MarkerSize',5);hold on;
% % surf(X,Y,ImpactDist);
% % caxis([min(ImpactDist(:))-.5*range(ImpactDist(:)),max(ImpactDist(:))]);
% % view([0 90]);

% % %     [rows1,columns1] = find( ImpactDist ~= 0); 
     idx2 = ImpactDist ~= 0;    

     Risk1 =  sum(dArea*ImpactDist(idx2).*PREM.layer1(idx2)*Population*dArea);
     
%      PREM_privacy = PREM.layer2;
     
     w1 = PREM.weights(1); w2 = PREM.weights(2);
     
%      privacy_cone = [X0;Y0] +  0.01*[cos(th);sin(th)];
%      
%      idx = inpolygon(X,Y,privacy_cone(1,:)',privacy_cone(2,:)');
%      
% %      privacy_risk1 = sum(PREM_privacy(idx))*dArea;
privacy_risk1 = 0;

     PositionRisk.cost = Risk1;
     PositionRisk.privacyCost = privacy_risk1;
     
     t0 = pathTree(PathNodeID).time;
     t1 = pathTree(PathNodeID).time + (length_segment/pathTree(PathNodeID).vel);
     
     REL = w1*0.5*(Risk0*(exp(-lambda*(t0)))+Risk1*(exp(-lambda*(t1))))*(t1-t0)...
            + w2*0.5*(privacy_risk0+privacy_risk1)*(t1-t0);

riskExposureAlongSegment = REL;
%     riskExposureAlongSegment = 0.5*(PREM_value1+PREM_value2)*length_segment;

riskExposureAlongPath = pathTree(PathNodeID).CumulativeCost + riskExposureAlongSegment;

[tr_test,T] = transitionTest(Risk0, Risk1, T);
    
end
