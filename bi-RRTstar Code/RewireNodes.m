% This file is created by U. Cem Kaya - 2018
%% REWIRING FUNCTION
function [pathTree,Tree] = RewireNodes(pathTree,Tree,extendedNode,nearNodes,...
    PREM,figHandle,mapSize,dX,dY,PURM,X_Grid,Y_Grid,Population,Constraints)


if length(nearNodes) > 1
    
    
    psiDot_Constraint = Constraints.psi_dot;
        
    for jj = 1:length(nearNodes)
        
        NearNode = pathTree(nearNodes{jj}.ID);
        psiNear = NearNode.psi;
        psi_Ext = atan2((extendedNode.y-NearNode.y),(extendedNode.x-NearNode.x));
        
        angDiff = psi_Ext - psiNear;
        
        if angDiff < -pi
            angDiff = angDiff + 2*pi;
        elseif angDiff > pi
            angDiff = angDiff - 2*pi;
        end
        
        dist = Dist([extendedNode.x extendedNode.y], [NearNode.x NearNode.y]);
        Velocity = NearNode.vel;
        delta_t_max = (dist/Velocity)*Constraints.delta_t(2);
        
        if (nearNodes{jj}.ID ~= pathTree(extendedNode.ID).parent)&&((abs(angDiff)/delta_t_max)<=psiDot_Constraint)
            [riskExposureAlongNearNode,riskExtendNode] = RiskExposureSegment(pathTree,PREM,[extendedNode.x extendedNode.y],nearNodes{jj}.ID,mapSize(:,1),mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population);
            
            if (riskExposureAlongNearNode < extendedNode.CumulativeCost)
                
%                 if plotBranches
% %                                 figure(figHandle)
% %                                 plot([pathTree(extendedNode.parent).x extendedNode.x] , [pathTree(extendedNode.parent).y extendedNode.y] ,'w-','LineWidth', 3.5);%delete old branch on the plot
% %                                 hold on;
% %                                 plot([nearNodes{jj}.x extendedNode.x] , [nearNodes{jj}.y extendedNode.y] ,'LineWidth', 2);
% %                                 hold on;
%                 end
                
                extendedNode.parent = nearNodes{jj}.ID;
                extendedNode.CumulativeCost = riskExposureAlongNearNode;
                extendedNode.cost = riskExtendNode.cost;
                extendedNode.privacyCost = riskExtendNode.privacyCost;
                extendedNode.totalDistance = pathTree(extendedNode.parent).totalDistance + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y]);
                extendedNode.time = pathTree(extendedNode.parent).time + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y])/pathTree(extendedNode.parent).vel;
                extendedNode.psi = psi_Ext;
            end
        end
    end

    Tree = rmedge(Tree,pathTree(extendedNode.ID).parent,extendedNode.ID);
    Tree = addedge(Tree,extendedNode.parent,extendedNode.ID);
                
    pathTree(extendedNode.ID).parent = extendedNode.parent;
    pathTree(extendedNode.ID).CumulativeCost = extendedNode.CumulativeCost;
    pathTree(extendedNode.ID).cost = extendedNode.cost;
    pathTree(extendedNode.ID).privacyCost = extendedNode.privacyCost;
    pathTree(extendedNode.ID).totalDistance = extendedNode.totalDistance;
    pathTree(extendedNode.ID).time = extendedNode.time;
    pathTree(extendedNode.ID).psi = extendedNode.psi;
    
    psiExtended = pathTree(extendedNode.ID).psi;
    
    for jj = 1:length(nearNodes)
         
        NearNode = pathTree(nearNodes{jj}.ID);       
        psiNear = atan2((NearNode.y-extendedNode.y),(NearNode.x-extendedNode.x));
        psi_Ext = extendedNode.psi;
        
        angDiff =  psiNear - psi_Ext;
        
        if angDiff < -pi
            angDiff = angDiff + 2*pi;
        elseif angDiff > pi
            angDiff = angDiff - 2*pi;
        end
        
        dist = Dist([extendedNode.x extendedNode.y], [NearNode.x NearNode.y]);
        Velocity = extendedNode.vel;
        delta_t_max = (dist/Velocity)*Constraints.delta_t(2);
        
        if (nearNodes{jj}.ID ~= pathTree(extendedNode.ID).parent)&&((abs(angDiff)/delta_t_max)<=psiDot_Constraint)
        
            [riskExposureAlongExtendedNode,riskNearNode] = RiskExposureSegment(pathTree,PREM,[nearNodes{jj}.x nearNodes{jj}.y],extendedNode.ID,mapSize(:,1),mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population);
            
            if (riskExposureAlongExtendedNode < NearNode.CumulativeCost)%&&(nearNodes(jj).parent ~= extendedNode.parent)
                
%                 parentNode = pathTree(nearNodes{jj}.parent);
                
                nearNodes{jj}.parent = extendedNode.ID;
                
                costChange = riskExposureAlongExtendedNode - NearNode.CumulativeCost;
%                 nearNodes{jj}.CumulativeCost = riskExposureAlongExtendedNode;             
%                 nearNodes{jj}.cost = riskNearNode;
                

                
                totalDistance = pathTree(nearNodes{jj}.parent).totalDistance + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y]);
                distanceChange = totalDistance - NearNode.totalDistance;
%                 nearNodes{jj}.totalDistance = totalDistance;
               
                

                
                TotalTime = pathTree(nearNodes{jj}.parent).time + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y])/pathTree(nearNodes{jj}.parent).vel;
                timeChange = TotalTime - NearNode.time;
%                 nearNodes{jj}.time = TotalTime;
                
                
         
                
                Tree = rmedge(Tree,pathTree(nearNodes{jj}.ID).parent,nearNodes{jj}.ID);
                Tree = addedge(Tree,extendedNode.ID,nearNodes{jj}.ID);
                
                if isdag(Tree)
                    
                    pathTree(nearNodes{jj}.ID).parent = nearNodes{jj}.parent;
                    pathTree(nearNodes{jj}.ID).CumulativeCost = riskExposureAlongExtendedNode;
                    pathTree(nearNodes{jj}.ID).cost = riskNearNode.cost;
                    pathTree(nearNodes{jj}.ID).privacyCost = riskNearNode.privacyCost;
                    pathTree(nearNodes{jj}.ID).totalDistance = totalDistance;
                    pathTree(nearNodes{jj}.ID).time = TotalTime;
                    
                    pathTree(nearNodes{jj}.ID).psi = psiNear;
                    
                    
                    % update all the children nodes starting from near node
                    pathTree = childrenNodesUpdate(pathTree,Tree,nearNodes{jj}.ID,costChange,distanceChange,timeChange);
                                     
% %                     figure(figHandle)
% %                     plot([parentNode.x nearNodes{jj}.x] , [parentNode.y nearNodes{jj}.y] ,'w-','LineWidth', 3.5);%delete old branch on the plot
% %                     hold on;
% %                     plot([extendedNode.x nearNodes{jj}.x] , [extendedNode.y nearNodes{jj}.y] ,'LineWidth', 2);
% %                     hold on;

% %                     break;

                else
                    
                     Tree = rmedge(Tree,extendedNode.ID,nearNodes{jj}.ID);
                     Tree = addedge(Tree,pathTree(nearNodes{jj}.ID).parent,nearNodes{jj}.ID);               

                end
                
            end
        end
    end

end


end