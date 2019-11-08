% This file is created by U. Cem Kaya - 2018
%% EXTEND FUNCTION
function [pathTree,Tree,extended] = Extend(nearest, target, stepSize,nearestNodeID,pathTree,Tree,PREM,mapSize,...
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
    
    delta_t_min = Constraints.delta_t(1);
    delta_t_max = Constraints.delta_t(2);
    psiDot_Constraint = Constraints.psi_dot;
    
    if ((abs(angDiff)/delta_t_max)<psiDot_Constraint)
    
        % Find sin and cos
        cosA = diffX/dist;
        sinA = diffY/dist;
        
        % Find new position after the step towards target, round them to
        % nearest integer
        if ( ((nearest(1) - target(1)) < 0) && ((nearest(2) - target(2)) < 0) )
            
            newPos1(1) = (nearest(1) + (0.5*stepSize * cosA));
            newPos1(2) = (nearest(2) + (0.5*stepSize * sinA));
            
            newPos2(1) = (nearest(1) + (stepSize * cosA));
            newPos2(2) = (nearest(2) + (stepSize * sinA));
            
        elseif ( ((nearest(1) - target(1)) > 0) && ((nearest(2) - target(2)) < 0) )
           
            newPos1(1) = (nearest(1) - (0.5*stepSize * cosA));
            newPos1(2) = (nearest(2) + (0.5*stepSize * sinA));
            
            newPos2(1) = (nearest(1) - (stepSize * cosA));
            newPos2(2) = (nearest(2) + (stepSize * sinA));
            
        elseif ( ((nearest(1) - target(1)) < 0) && ((nearest(2) - target(2)) > 0) )
            
            newPos1(1) = (nearest(1) + (0.5*stepSize * cosA));
            newPos1(2) = (nearest(2) - (0.5*stepSize * sinA));
            
            newPos2(1) = (nearest(1) + (stepSize * cosA));
            newPos2(2) = (nearest(2) - (stepSize * sinA));
        else
            
            newPos1(1) = (nearest(1) - (0.5*stepSize * cosA));
            newPos1(2) = (nearest(2) - (0.5*stepSize * sinA));
            
            newPos2(1) = (nearest(1) - (stepSize * cosA));
            newPos2(2) = (nearest(2) - (stepSize * sinA));
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
            
            midPoint_ID = length(pathTree);
            
            %% Check Second Point
            delta_dist_2 = Dist(newPos1,newPos2);
            TotalLength_2 = TotalLength_1 + delta_dist_2;
        
            [riskExposureAlongSegment_2,PositionRisk_2,collision_2] = collCheck(newPos2,midPoint_ID,pathTree,PREM,mapSize,...
                dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength_2,obstacles);
            
             if (collision_2~=true)
                 
                 [pathTree,Tree] = AddNode(pathTree,Tree, newPos2, midPoint_ID,riskExposureAlongSegment_2,PositionRisk_2,delta_dist_2,Velocity);
                 
                 extended = newPos2;
             end
        end
    
    else

        max_turn_angle = delta_t_min*psiDot_Constraint;
           
        if  (angDiff>0) % add turn angle to close the gap
            
            newPos1(1) = (nearest(1) + (0.5*stepSize * cos(psiNearest + max_turn_angle)));
            newPos1(2) = (nearest(2) + (0.5*stepSize * sin(psiNearest + max_turn_angle)));
            
            newPos2(1) = (newPos1(1) + (0.5*stepSize * cos(psiNearest + 2*max_turn_angle)));
            newPos2(2) = (newPos1(2) + (0.5*stepSize * sin(psiNearest + 2*max_turn_angle)));
            
        else % remove turn angle to close the gap
            
            newPos1(1) = (nearest(1) + (0.5*stepSize * cos(psiNearest - max_turn_angle)));
            newPos1(2) = (nearest(2) + (0.5*stepSize * sin(psiNearest - max_turn_angle)));
            
            newPos2(1) = (newPos1(1) + (0.5*stepSize * cos(psiNearest - 2*max_turn_angle)));
            newPos2(2) = (newPos1(2) + (0.5*stepSize * sin(psiNearest - 2*max_turn_angle)));
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
            
            midPoint_ID = length(pathTree);
            
            %% Check Second Point
            delta_dist_2 = Dist(newPos1,newPos2);
            TotalLength_2 = TotalLength_1 + delta_dist_2;
        
            [riskExposureAlongSegment_2,PositionRisk_2,collision_2] = collCheck(newPos2,midPoint_ID,pathTree,PREM,mapSize,...
                dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength_2,obstacles);
            
             if (collision_2~=true)
                 
                 [pathTree,Tree] = AddNode(pathTree,Tree, newPos2, midPoint_ID,riskExposureAlongSegment_2,PositionRisk_2,delta_dist_2,Velocity);
                 
                 extended = newPos2;
             end
        end
    end
    
end