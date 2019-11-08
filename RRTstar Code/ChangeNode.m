% This file is created by U. Cem Kaya - 2018
%% CHANGE NODE FUNCTION
function pathTree = ChangeNode(pathTree, extended, NodeID,parentNodeID,CumulativeCost,PositionRisk,Velocity)
    
    dist = Dist([pathTree(parentNodeID).x pathTree(parentNodeID).y],extended);
    % Get the length of the tree
    
    % Add new element to the tree
    pathTree(NodeID).x = extended(1);
    pathTree(NodeID).y = extended(2);
    pathTree(NodeID).parent = parentNodeID;
    pathTree(NodeID).CumulativeCost = CumulativeCost;
    pathTree(NodeID).cost = PositionRisk.cost;
    pathTree(NodeID).privacyCost = PositionRisk.privacyCost;
    pathTree(NodeID).totalDistance = pathTree(parentNodeID).totalDistance + dist;
    
    pathTree(NodeID).psi = heading(pathTree(parentNodeID),pathTree(NodeID));
    pathTree(NodeID).vel = Velocity;
    pathTree(NodeID).time = pathTree(parentNodeID).time + dist/Velocity;
    
end