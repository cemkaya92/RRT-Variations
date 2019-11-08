% This file is created by U. Cem Kaya - 2018
function [FinalSmoothedPath,smoothTotal_distance,smooth_REL,pathTree1,pathTree2] = LocalBiasSmoothing(Path,...
    PREM,mapSize,dX,dY,PURM,X_Grid,Y_Grid,Population,Velocity,figHandle,RiskThreshold,obstacles,...
    NumItereation,pathTree1,Tree1,pathTree2,Tree2,stepSize,ResMapSize,Constraints,gama,...
    start,finish,NodeIDsuccess_1,NodeIDsuccess_2)

    
% %     writerObj = VideoWriter('PathSmoothing.avi');
% %     writerObj.FrameRate = 12;
% %     open(writerObj);

% % %     figure(figHandle)
% % %     str = get(legend(gca),'String');

%     legend('off')
%     str{1,end+1} = 'Smooth Path';
    
    for i = 1:NumItereation
%         figure(figHandle);
%         title(['Iter = ' num2str(i)]);
        
        randomNodeID =  2 + randi(length(Path)-3,1,1);
        randomNode = Path(randomNodeID);
        parentNode = Path(randomNodeID-1);
        parentNodeID = parentNode.ID;
        childNode = Path(randomNodeID+1);
        childNodeID = childNode.ID; % either in tree1 or tree2

        q = [randomNode.x; randomNode.y];
        q1 = [parentNode.x; parentNode.y];
        q2 = [childNode.x; childNode.y];
        
        r_max = Dist(q1,q2)/2;
        
        q_temp = ((q1+q2)/2)-q;
        
        q_temp_mag = sqrt(q_temp(1)^2 + q_temp(2)^2);
        
        q_rand = q + (q_temp/q_temp_mag)*(-r_max + 2*r_max*rand);
       
        if (randomNode.TreeNo == 1)&&(childNode.TreeNo == 1)&&(parentNode.TreeNo == 1)
            
            [pathTree1,Tree1,extended_1] = Extend_1(q, q_rand, stepSize,randomNode.ID,pathTree1,Tree1,PREM,ResMapSize,...
                dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,obstacles,Constraints,Velocity);
            
            if (extended_1 ~= 0) 
%                 plot([pathTree1(pathTree1(end).parent).x pathTree1(end).x] , [pathTree1(pathTree1(end).parent).y pathTree1(end).y] ,'b','LineWidth', 2);
%             hold on;
                NumOfNodes_1 = length(pathTree1);
                KDtree1 = KDTreeSearcher([pathTree1.x;pathTree1.y]');
                r_1 = gama*(log2(NumOfNodes_1)/(NumOfNodes_1))^(1/4);
                nearNodes_1=Near(pathTree1,extended_1,r_1,KDtree1);
                extendedNode_1 = pathTree1(end);
                extendedNode_1.ID = length(pathTree1);
            
%                 childNode_CumCost = pathTree1(childNodeID).CumulativeCost;
%                 childNode_time = pathTree1(childNodeID).time;
%                 childNode_totalDist = pathTree1(childNodeID).totalDistance;
                
            [pathTree1,Tree1] = RewireNodes(pathTree1,Tree1,extendedNode_1,nearNodes_1,...
                                PREM,figHandle,mapSize,dX,dY,PURM,X_Grid,Y_Grid,...
                                Population,Constraints);
%                 childThroughExt_CumCost = pathTree1(childNodeID).CumulativeCost;
%                 childThroughExt_time = pathTree1(childNodeID).time;
%                 childThroughExt_totalDist = pathTree1(childNodeID).totalDistance;
            end
            
            if (pathTree1(childNodeID).parent ~= randomNode.ID)   % then path changed
% %                 costChange = childThroughExt_CumCost - childNode_CumCost;
% %                 timeChange = childThroughExt_time - childNode_time;
% %                 distanceChange = childThroughExt_totalDist - childNode_totalDist;
% %                 Path = Update_from_Node(Path,randomNodeID+1,costChange,timeChange,distanceChange);
                [Path, ~,~,~] = bidirectionalBacktrack(pathTree1,pathTree2,start,finish,NodeIDsuccess_1(end),NodeIDsuccess_2(end),...
                     figHandle,Velocity,PURM,1,Path);

%                  l =  plot([Path(:).x]',[Path(:).y]','r','LineWidth', 2);

%                  frm = getframe;
%                  img = frame2im(frm);
%                  writeVideo(writerObj, img);
%                  delete(l);
            end
            
        elseif (randomNode.TreeNo == 2)&&(childNode.TreeNo == 2)&&(parentNode.TreeNo == 2)
            
            [pathTree2,Tree2,extended_2] = Extend_1(q, q_rand, stepSize,randomNode.ID,pathTree2,Tree2,PREM,ResMapSize,...
                dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,obstacles,Constraints,Velocity);
            
            if (extended_2 ~= 0) 
%                 plot([pathTree2(pathTree2(end).parent).x pathTree2(end).x] , [pathTree2(pathTree2(end).parent).y pathTree2(end).y] ,'r','LineWidth', 2);
%                          hold on;
                NumOfNodes_2 = length(pathTree2);
                KDtree2 = KDTreeSearcher([pathTree2.x;pathTree2.y]');
                r_2 = gama*(log2(NumOfNodes_2)/(NumOfNodes_2))^(1/4);
                nearNodes_2=Near(pathTree2,extended_2,r_2,KDtree2);
                extendedNode_2 = pathTree2(end);
                extendedNode_2.ID = length(pathTree2);
                
%                 childNode_CumCost = pathTree2(parentNodeID).CumulativeCost;
%                 childNode_time = pathTree2(parentNodeID).time;
%                 childNode_totalDist = pathTree2(parentNodeID).totalDistance;
            
            [pathTree2,Tree2] = RewireNodes(pathTree2,Tree2,extendedNode_2,nearNodes_2,...
                                PREM,figHandle,mapSize,dX,dY,PURM,X_Grid,Y_Grid,...
                                Population,Constraints);
                            
%                 childThroughExt_CumCost = pathTree2(parentNodeID).CumulativeCost;
%                 childThroughExt_time = pathTree2(parentNodeID).time;
%                 childThroughExt_totalDist = pathTree2(parentNodeID).totalDistance;
            end
            
            
            if (pathTree2(parentNodeID).parent ~= randomNode.ID)   % then path changed
%                 costChange = childThroughExt_CumCost - childNode_CumCost;
%                 timeChange = childThroughExt_time - childNode_time;
%                 distanceChange = childThroughExt_totalDist - childNode_totalDist;
%                 Path = Update_from_Node(Path,randomNodeID+1,costChange,timeChange,distanceChange);
                [Path, ~,~,~] = bidirectionalBacktrack(pathTree1,pathTree2,start,finish,NodeIDsuccess_1,NodeIDsuccess_2,...
                     figHandle,Velocity,PURM,1,Path);
                 
%                  l =  plot([Path(:).x]',[Path(:).y]','r','LineWidth', 2);

%                  frm = getframe;
%                  img = frame2im(frm);
%                  writeVideo(writerObj, img);
%                  delete(l);
            end
 
        end
        



% %                                                 l =  plot([Path(:).x]',[Path(:).y]','m','LineWidth', 2);
% %                     
% %                     
% %                     
% %                     %                             frm = getframe;
% %                     %                             img = frame2im(frm);
% %                     %                             writeVideo(writerObj, img);
% %                                                 delete(l);
                    

    
    end
    

    
    
    FinalSmoothedPath = Path;
    smoothTotal_distance = Path(end).totalDistance;
    smooth_REL = Path(end).CumulativeCost;

end