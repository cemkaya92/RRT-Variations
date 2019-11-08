% This file is created by U. Cem Kaya - 2018
%% ADD NODE FUNCTION
function [pathTree,Tree] = AddNode(pathTree,Tree, extended, parentNodeID,CumulativeCost,PositionRisk,delta_dist,Velocity)
    
    % Get the length of the tree
    tlength = length(pathTree);
    
    % Add new element to the tree
    Tree = addnode(Tree,1);
        
    pathTree(tlength+1).x = extended(1);
    pathTree(tlength+1).y = extended(2);
    pathTree(tlength+1).parent = parentNodeID;
    pathTree(parentNodeID).children = [pathTree(parentNodeID).children tlength+1];
    pathTree(tlength+1).CumulativeCost = CumulativeCost;
    pathTree(tlength+1).cost = PositionRisk.cost;
    pathTree(tlength+1).privacyCost = PositionRisk.privacyCost;
    pathTree(tlength+1).ID = tlength+1;
    Tree = addedge(Tree,parentNodeID,tlength+1);
    pathTree(tlength+1).totalDistance = pathTree(parentNodeID).totalDistance + delta_dist;
    
    pathTree(tlength+1).psi = heading(pathTree(parentNodeID),pathTree(tlength+1));
    pathTree(tlength+1).vel = Velocity;
    pathTree(tlength+1).time = pathTree(parentNodeID).time + delta_dist/Velocity;
    
    pathTree(tlength+1).TreeNo = pathTree(tlength).TreeNo;
    
end