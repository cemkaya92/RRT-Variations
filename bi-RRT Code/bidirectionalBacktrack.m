% This file is created by U. Cem Kaya - 2018
%% FUNCTION TO BIDIRECTIONAL BACKTRACK

function [Path, riskExposureLevel,total_distance,MaxRiskRate] = bidirectionalBacktrack(pathTree1,pathTree2,start,finish,NodeIDsuccess_1,NodeIDsuccess_2,...
                     figHandle,V_tar1,PURM,success,finalPath)
                 
  
riskExposureLevel = 0;
total_distance = 0;
MaxRiskRate = 0;
 % Check if goal was reached
    if (success == true)
        %% Select the path having the minimum cost
        for jj =1:length(NodeIDsuccess_1)
             successNodeCosts(jj)= (pathTree1(NodeIDsuccess_1(jj)).CumulativeCost + pathTree2(NodeIDsuccess_2(jj)).CumulativeCost);
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
        
        
        
% %         %% Add Bi-directional tree at the end of tree1
% %         pathTree2(NodeIDsuccess_2(ID)).x = pathTree1(NodeIDsuccess_1(ID)).x;
% %         pathTree2(NodeIDsuccess_2(ID)).y = pathTree1(NodeIDsuccess_1(ID)).y;
%         

        currNode_2 = pathTree2(NodeIDsuccess_2(ID));
        parentID_2 = currNode_2.parent;
        
%         btX_2 = pathTree2(NodeIDsuccess_2(ID)).x;
%         btY_2 = pathTree2(NodeIDsuccess_2(ID)).y;
%         figure(figHandle);
%         plot([btX_2 pathTree2(parentID_2).x] , [btY_2 pathTree2(parentID_2).y] , 'm--', 'LineWidth', 3);
%         hold on;
     if (parentID_2 == 1 )
            Path(end+1) = pathTree2(1);
     else
% %         currNode_2 = pathTree2(parentID_2);
% %         
% %         btX_2 = pathTree2(parentID_2).x;
% %         btY_2 = pathTree2(parentID_2).y;
% %         
        n2 = n1+2;        
        
        Path(n2) = currNode_2;
        % find parent of current node
        parentID_2 = currNode_2.parent;
        
        dist = Dist([Path(n2).x Path(n2).y],[Path(n2-1).x Path(n2-1).y]);
        % Add the cost of pathTree2 nodes 
        Path(n2).CumulativeCost = Path(n2-1).CumulativeCost;
        Path(n2).totalDistance = Path(n2-1).totalDistance + dist;
        Path(n2).time = Path(n2-1).time + dist/currNode_2.vel;
        Path(n2).psi = heading(currNode_2,Path(n2-1));
        
         % Change current node to its parent
         childNode_2 = currNode_2;
         currNode_2 = pathTree2(parentID_2);
         
         n2 = n2 + 1;
         
        % Backtrack the best path
        while (1)
            Path(n2) = currNode_2;
            % find parent of current node
            parentID_2 = currNode_2.parent;
            
            Path(n2).psi = heading(currNode_2,pathTree2(parentID_2));
            % Plot backtrack in red line
%             figure(figHandle);
%             plot([btX_2 pathTree2(parentID_2).x] , [btY_2 pathTree2(parentID_2).y] , 'm--', 'LineWidth', 3);
%             hold on;
            
            % Add the cost of pathTree2 nodes 
            Path(n2).CumulativeCost = Path(n2-1).CumulativeCost + childNode_2.CumulativeCost - currNode_2.CumulativeCost;
            Path(n2).totalDistance = Path(n2-1).totalDistance + childNode_2.totalDistance - currNode_2.totalDistance;
            Path(n2).time = Path(n2-1).time + childNode_2.time - currNode_2.time;
            % Change current node to its parent
            
                       
              childNode_2 = currNode_2;
            % Change current node to its parent
            currNode_2 = pathTree2(parentID_2);
            btX_2 = currNode_2.x;
            btY_2 = currNode_2.y;

            
            % If startpoint was reached, break out of the loop
            if ( (btX_2 == finish(1)) && (btY_2 == finish(2)) )
                break;
            end
            
            n2 = n2 + 1;
            
            if n2 > length(pathTree2)
                Path = finalPath;
                break;
            end
        end
        Path(end+1) = pathTree2(1); % add finish point to the end
     end
        
    
        % Assign Heading angle and Velocity
% %         for ii = 1:length(Path)-1
% % %             Path(ii).psi = heading(Path(ii),Path(ii+1));
% % %             Path(ii).vel = V_tar1;
% % %             Path(ii).ax = 0; Path(ii).ay = 0; 
% % % %             x1 = [Path(ii).x Path(ii).y];
% % % %             x2 = [Path(ii+1).x Path(ii+1).y];
% % % %             dist = Dist(x1,x2);
% % % %             Path(ii+1).totalDistance = Path(ii).totalDistance + dist;
% % % %             Path(ii+1).time = Path(ii).time + dist/Path(ii).vel;
% %             
% %             %         riskExposureLevel = RiskExposure(prem,Path);
% % %             [riskExposureAlongSegment,PositionRisk] = RiskExposureSegment(Path,PREM_value,[nearNodes(jj).x nearNodes(jj).y],extendedNode.ID,mapSize(:,1),mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population);
% %             
% % % %             Path(ii+1).CumulativeCost = Path(ii).CumulativeCost + 0.5*(Path(ii).cost + Path(ii+1).cost)*(exp(-lambda*Path(ii).time)-exp(-lambda*Path(ii+1).time))/lambda; % change this
% %         end
%         Path(end).psi = 0;
%         Path(end).vel = 0;
        % Add zero accelerations too
%         Path(end).ax = 0; Path(end).ay = 0; 
            x1 = [Path(end-1).x Path(end-1).y];
            x2 = [Path(end).x Path(end).y];
            dist = Dist(x1,x2);
            Path(end).totalDistance = Path(end-1).totalDistance + dist;
            Path(end).time = Path(end-1).time + dist/Path(end-1).vel;            
        
%             [riskExposureAlongSegment,PositionRisk] = RiskExposureSegment(Path,PREM_value,x2,(length(Path)-1),mapSize(:,1),mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population);
        Path(end).CumulativeCost = Path(end-1).CumulativeCost;
        
        riskExposureLevel = Path(end).CumulativeCost;
        total_distance = Path(end).totalDistance;
        MaxRiskRate = max([Path(:).cost]);
       
    else
        Path = finalPath;
%         fprintf('After %d iterations, goal was not reached...\n', iter);
    end

end