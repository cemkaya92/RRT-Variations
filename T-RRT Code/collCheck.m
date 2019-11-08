% This file is created by U. Cem Kaya - 2018
%% COLLISION CHECK FUNCTION
function [riskExposureAlongPath,PositionRisk,collision,T] = collCheck(newPosition,nearestNodeID,pathTree,PREM,mapSize,...
    dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength,obstacles,T)

LengthThreshold = RiskThreshold.TotalLength;
CumulativeRiskThreshold = RiskThreshold.Cumulative;
ExposureRateThreshold = RiskThreshold.ExposureRate;

p1 = [pathTree(nearestNodeID).x; pathTree(nearestNodeID).y]; 
p2 = [newPosition(1); newPosition(2)];

r = sqrt(sum((p2-p1).^2))/2; % radius of the circle enclosing the straight line
center_of_circle = (p1+p2)/2;
x_center = center_of_circle(1);
y_center = center_of_circle(2);

PolygonCheck = false;

if(~isempty(obstacles))

    OuterCircleCheck = ((sum(([x_center;y_center]-[obstacles.center]).^2))<([obstacles.outerRadius]+r).^2);
    
    [~,ObsID] = find(OuterCircleCheck == true);
    
    if (~isempty(ObsID))
        [~,InsideObstCircleID] = find((([obstacles.outerRadius]-r).^2>(sum(([x_center;y_center]-[obstacles.center]).^2))) == true);
        IntersectingCirclesID = setdiff(ObsID,InsideObstCircleID);
        
        for j = 1:length(InsideObstCircleID)
            PolygonCheck = inpolygon(newPosition(1),newPosition(2),obstacles(InsideObstCircleID(j)).pos(1,:)',obstacles(InsideObstCircleID(j)).pos(2,:)');
            if PolygonCheck == true
                break;
            end
        end
        if PolygonCheck == false
            for j = 1:length(IntersectingCirclesID)
                
                PolygonCheck = inpolygon(newPosition(1),newPosition(2),obstacles(IntersectingCirclesID(j)).pos(1,:)',obstacles(IntersectingCirclesID(j)).pos(2,:)');
                if PolygonCheck == true
                    break;
                end
                
            end
        end
    end
 
end

if ((TotalLength>LengthThreshold)||PolygonCheck||(newPosition(1)<mapSize(1,1))||(newPosition(1)>mapSize(2,1))...
        ||(newPosition(2)<mapSize(1,2))||(newPosition(2)>mapSize(2,2)))

    collision = true;
    riskExposureAlongPath = NaN;
    PositionRisk = NaN;
else
    
    [riskExposureAlongPath,PositionRisk,tr_test,T] = RiskExposureSegment(pathTree,PREM,newPosition,...
        nearestNodeID,mapSize(:,1),mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population,T);
    
%     if ((PositionRisk.cost>ExposureRateThreshold) || (riskExposureAlongPath>CumulativeRiskThreshold))
    if ((PositionRisk.cost>ExposureRateThreshold)) || (tr_test == 0)
        collision = true;
    else
        collision = false;
        
        if T.range < PositionRisk.cost
            T.range = PositionRisk.cost;
        end
    end

%     collision = false;
    
end

end