% This file is created by U. Cem Kaya - 2018
%% FUNCTION TO FIND NEAR NODES IN THE PROXIMITY OF EXTENDED NODE
function nearNodes=Near(pathTree,extended,r,KDtree)

    
    NodeID = rangesearch(KDtree,extended,r);
% counter = 0;
% % % nodeDistArray = zeros(length(pathTree),1);
% % % for i=1:length(pathTree)
% % %    
% % %         tempNode(1) = pathTree(i).x;
% % %         tempNode(2) = pathTree(i).y;
% % %         nodeDistArray(i) = Dist(tempNode, extended);
% % % end
% % % % %     dist_bw_Near_and_Path = Dist(extended,[pathTree(i).x pathTree(i).y]);
% % %     
% % % % %     if dist_bw_Near_and_Path <=r && dist_bw_Near_and_Path > 0
% % % % %         counter = counter +1;
% % % % %         nearNodes(counter).x = pathTree(i).x;
% % % % %         nearNodes(counter).y = pathTree(i).y;
% % % % %         nearNodes(counter).parent = pathTree(i).parent;
% % % % %         nearNodes(counter).CumulativeCost = pathTree(i).CumulativeCost;
% % % % %         nearNodes(counter).cost = pathTree(i).cost;
% % % % %         nearNodes(counter).time = pathTree(i).time;
% % % % %         nearNodes(counter).totalDistance = pathTree(i).totalDistance;
% % % % %         nearNodes(counter).ID = i;
% % %         
% % %     
% % % % nodeDistArray = sort(nodeDistArray);
% % % 
% % % NodeID = find(nodeDistArray <= r);
 
lengthNodeID = length(NodeID{:});

nearNodes = cell(lengthNodeID,1);

if lengthNodeID > 1
    
    for id = 1:lengthNodeID
        nearNodes{id,1} = pathTree(NodeID{1}(id));
    end
    
else
    
    nearNodes = {};
    
end
        

end 