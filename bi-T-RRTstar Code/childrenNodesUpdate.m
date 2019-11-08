% This file is created by U. Cem Kaya - 2018
function PathTree = childrenNodesUpdate(PathTree,Tree,startingNodeID,costChange,distanceChange,timeChange)

      childrenNodeIDs = dfsearch(Tree,startingNodeID);

%     sortedPath = struct2cell(Tree);
%     sz = size(sortedPath);
%     
%     % convert to a matrix
%     sortedPath = reshape(sortedPath,sz(1),[]);
%     %make each field a column
%     sortedPath = sortedPath';
%     
%     %sort by the field (parent ID 3rd field)
%     sortedPath = sortrows(sortedPath,3);
      

for j = 2:length(childrenNodeIDs)
    
    id = childrenNodeIDs(j);
    
% %     if PathTree(id).CumulativeCost + costChange - PathTree(PathTree(id).parent).CumulativeCost < 0
% %         lookAtThis = 0;
% %     end
% %     
% %     if PathTree(id).totalDistance + distanceChange - PathTree(PathTree(id).parent).totalDistance < 0
% %         lookAtThis = 0;
% %     end
% %     
% %     if PathTree(id).time + timeChange - PathTree(PathTree(id).parent).time < 0
% %         lookAtThis = 0;
% %     end
    
    PathTree(id).CumulativeCost = PathTree(id).CumulativeCost + costChange;
    PathTree(id).time = PathTree(id).time + timeChange;
    PathTree(id).totalDistance = PathTree(id).totalDistance + distanceChange;
    

       
end

end