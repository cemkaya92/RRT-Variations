% This file is created by U. Cem Kaya - 2018
%% FUNCTION TO BIDIRECTIONAL BACKTRACK

function [Path, riskExposureLevel,total_distance,MaxRiskRate] = Backtrack(pathTree1,start,finish,NodeIDsuccess_1,...
                     figHandle,V_tar1,PURM,success,finalPath)
                 
  
riskExposureLevel = 0;
total_distance = 0;
MaxRiskRate = 0;
 % Check if goal was reached
    if (success == true)
        %% Select the path having the minimum cost
        for jj =1:length(NodeIDsuccess_1)
             successNodeCosts(jj)= (pathTree1(NodeIDsuccess_1(jj)).CumulativeCost );
        end    
        [minPathCost,ID] = min(successNodeCosts);
        
        btX_1 = pathTree1(NodeIDsuccess_1(ID)).x;
        btY_1 = pathTree1(NodeIDsuccess_1(ID)).y;
               
%         currNode = pathTree(length(pathTree));
        currNode_1 = pathTree1(NodeIDsuccess_1(ID));
                    
                
        n1 = 1;
        % Backtrack the best path
        while (1)
            Path(n1) = currNode_1;
            % find parent of current node
            parentID_1 = currNode_1.parent;
            % Plot backtrack in red line
%             figure(figHandle);
%             plot([btX_1 pathTree1(parentID_1).x] , [btY_1 pathTree1(parentID_1).y] , 'm--', 'LineWidth', 3);
%             hold on;
              
            
            % Change current node to its parent
            currNode_1 = pathTree1(parentID_1);
            btX_1 = currNode_1.x;
            btY_1 = currNode_1.y;

               % If startpoint was reached, break out of the loop
            if ( (btX_1 == start(1)) && (btY_1 == start(2)) )
                break;
            end         
           
            n1 = n1 + 1;
            
            if n1 > length(pathTree1)
                Path = finalPath;
                break;
            end
        end
        Path(end+1) = pathTree1(1); % add starting point to the end
        Path = fliplr(Path); % reverse the order
        
        riskExposureLevel = Path(end).CumulativeCost;
        total_distance = Path(end).totalDistance;
        MaxRiskRate = max([Path(:).cost]);
       
    else
        Path = finalPath;
%         fprintf('After %d iterations, goal was not reached...\n', iter);
    end

end