% This file is created by U. Cem Kaya - 2018
%% EXTEND FUNCTION
function [pathTree,Tree,extended] = Extend_1(nearest, target, stepSize,nearestNodeID,pathTree,Tree,PREM,mapSize,...
    dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,obstacles,Constraints,Velocity)   
    
    % Find diffX and diffY
    diffX = abs(nearest(1) - target(1));
    diffY = abs(nearest(2) - target(2));
    
    % Find distance between them
    dist = Dist(nearest, target);
   
    psiNearest =  pathTree(nearestNodeID).psi; % between 
    psiTarget = atan2((target(2)-nearest(2)),(target(1)-nearest(1)));
    
    angDiff = psiTarget-psiNearest;
    
    if angDiff < -pi
        angDiff = angDiff + 2*pi;
    elseif angDiff > pi
        angDiff = angDiff - 2*pi;
    end
    
    delta_t_min = (dist/Velocity)*Constraints.delta_t(1);
    delta_t_max = (dist/Velocity)*Constraints.delta_t(2);
    psiDot_Constraint = Constraints.psi_dot;
    
    if ((abs(angDiff)/delta_t_max)<=psiDot_Constraint)&&(dist <= stepSize)
        
        
        TotalLength_1 = pathTree(nearestNodeID).totalDistance + dist;
        
        [riskExposureAlongSegment_1,PositionRisk_1,collision_1] = collCheck(target,nearestNodeID,pathTree,PREM,mapSize,...
            dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength_1,obstacles);
        
        if (collision_1==true)
            extended = 0;
        else
            
            extended = target;
            
            [pathTree,Tree] = AddNode(pathTree,Tree, target, nearestNodeID,riskExposureAlongSegment_1,PositionRisk_1,dist,Velocity);
            
           
        end
        
        
    elseif ((abs(angDiff)/delta_t_max)>psiDot_Constraint)&&(dist <= stepSize)
        
        max_turn_angle = delta_t_min*psiDot_Constraint;
           
        if  (angDiff>0) % add turn angle to close the gap
            
            newPos1(1) = (nearest(1) + (dist * cos(psiNearest + max_turn_angle)));
            newPos1(2) = (nearest(2) + (dist * sin(psiNearest + max_turn_angle)));

            
        else % remove turn angle to close the gap
            
            newPos1(1) = (nearest(1) + (dist * cos(psiNearest - max_turn_angle)));
            newPos1(2) = (nearest(2) + (dist * sin(psiNearest - max_turn_angle)));
            
        end
        
       
        TotalLength_1 = pathTree(nearestNodeID).totalDistance + dist;
        
        [riskExposureAlongSegment_1,PositionRisk_1,collision_1] = collCheck(newPos1,nearestNodeID,pathTree,PREM,mapSize,...
            dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength_1,obstacles);
        
        if (collision_1==true)
            extended = 0;
        else
            
            extended = newPos1;
            
            [pathTree,Tree] = AddNode(pathTree,Tree, newPos1, nearestNodeID,riskExposureAlongSegment_1,PositionRisk_1,dist,Velocity);
            
            
        end
        
    elseif ((abs(angDiff)/delta_t_max)<=psiDot_Constraint)&&(dist > stepSize)
        % Find sin and cos
        cosA = diffX/dist;
        sinA = diffY/dist;
        
        % Find new position after the step towards target, round them to
        % nearest integer
        if ( ((nearest(1) - target(1)) < 0) && ((nearest(2) - target(2)) < 0) )
            
            
            newPos1(1) = (nearest(1) + (stepSize * cosA));
            newPos1(2) = (nearest(2) + (stepSize * sinA));
            
        elseif ( ((nearest(1) - target(1)) > 0) && ((nearest(2) - target(2)) < 0) )
           
            
            newPos1(1) = (nearest(1) - (stepSize * cosA));
            newPos1(2) = (nearest(2) + (stepSize * sinA));
            
        elseif ( ((nearest(1) - target(1)) < 0) && ((nearest(2) - target(2)) > 0) )
            
            
            newPos1(1) = (nearest(1) + (stepSize * cosA));
            newPos1(2) = (nearest(2) - (stepSize * sinA));
        else
            
            
            newPos1(1) = (nearest(1) - (stepSize * cosA));
            newPos1(2) = (nearest(2) - (stepSize * sinA));
        end
        delta_dist_1 = Dist(nearest,newPos1);
        TotalLength_1 = pathTree(nearestNodeID).totalDistance + delta_dist_1;
        
        [riskExposureAlongSegment_1,PositionRisk_1,collision_1] = collCheck(newPos1,nearestNodeID,pathTree,PREM,mapSize,...
            dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength_1,obstacles);
        
        if (collision_1==true)
            extended = 0;
        else
            
            extended = newPos1;
            
            [pathTree,Tree] = AddNode(pathTree,Tree, newPos1, nearestNodeID,riskExposureAlongSegment_1,PositionRisk_1,delta_dist_1,Velocity);
            
           
        end
    else

        max_turn_angle = delta_t_min*psiDot_Constraint;
           
        if  (angDiff>0) % add turn angle to close the gap
            
            newPos1(1) = (nearest(1) + (stepSize * cos(psiNearest + max_turn_angle)));
            newPos1(2) = (nearest(2) + (stepSize * sin(psiNearest + max_turn_angle)));

            
        else % remove turn angle to close the gap
            
            newPos1(1) = (nearest(1) + (stepSize * cos(psiNearest - max_turn_angle)));
            newPos1(2) = (nearest(2) + (stepSize * sin(psiNearest - max_turn_angle)));
            
        end
        
        delta_dist_1 = Dist(nearest,newPos1);
        TotalLength_1 = pathTree(nearestNodeID).totalDistance + delta_dist_1;
        
        [riskExposureAlongSegment_1,PositionRisk_1,collision_1] = collCheck(newPos1,nearestNodeID,pathTree,PREM,mapSize,...
            dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength_1,obstacles);
        
        if (collision_1==true)
            extended = 0;
        else
            
            extended = newPos1;
            
            [pathTree,Tree] = AddNode(pathTree,Tree, newPos1, nearestNodeID,riskExposureAlongSegment_1,PositionRisk_1,delta_dist_1,Velocity);
            
            
        end
    end
    
end