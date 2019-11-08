% This file is created by U. Cem Kaya - 2018
function [FinalSmoothedPath,smoothTotal_distance,smooth_REL,pathTree1,pathTree2] = LocalBiasingSmoothing(Path,...
    PREM,mapSize,dX,dY,PURM,X_Grid,Y_Grid,Population,Velocity,figHandle,RiskThreshold,obstacles,...
    NumItereation,pathTree1,Tree1,pathTree2,Tree2)

    
    tempPath = Path;
    
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
        childNode = Path(randomNodeID+1);
        
        q = [randomNode.x; randomNode.y];
        q1 = [parentNode.x; parentNode.y];
        q2 = [childNode.x; childNode.y];
        
        r_max = Dist(q1,q2)/2;
        
        q_temp = ((q1+q2)/2)-q;
        
        q_temp_mag = sqrt(q_temp(1)^2 + q_temp(2)^2);
        
        q_rand = q + (q_temp/q_temp_mag)*(-r_max + 2*r_max*rand);
%         q_rand = q + (q_temp/q_temp_mag)*(0 + q_temp_mag*rand);
        
        TotalLength = Path(end).totalDistance;
        
        [RELfrom_q1_to_q_rand,PositionRisk_q_rand,collision] = collCheck(q_rand,(randomNodeID-1),tempPath,PREM,mapSize,...
    dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength,obstacles);

% %         [RELfrom_q1_to_q_rand,PositionRisk_q_rand] = RiskExposureSegment(tempPath,PREM_value,q_rand,...
% %             (randomNodeID-1),mapSize(:,1),mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population);

        if collision == false
            
        
            tempPath = ChangeNode(tempPath, q_rand, randomNodeID,(randomNodeID-1),RELfrom_q1_to_q_rand,PositionRisk_q_rand,Velocity);
            
            [RELfrom_q_rand_to_q2,PositionRisk_q2,collision2] = collCheck(q2,(randomNodeID),tempPath,PREM,mapSize,...
    dX,dY,PURM,X_Grid,Y_Grid,Population,RiskThreshold,TotalLength,obstacles);

% %             [RELfrom_q_rand_to_q2,PositionRisk_q2] = RiskExposureSegment(tempPath,PREM_value,q2,...
% %                 (randomNodeID),mapSize(:,1),mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population);
            
            if collision2 == false
                tempPath = ChangeNode(tempPath, q2, (randomNodeID+1),randomNodeID,RELfrom_q_rand_to_q2,PositionRisk_q2,Velocity);
                
                costChange = tempPath(randomNodeID+1).CumulativeCost - Path(randomNodeID+1).CumulativeCost;
                timeChange = tempPath(randomNodeID+1).time - Path(randomNodeID+1).time;
                distanceChange = tempPath(randomNodeID+1).totalDistance - Path(randomNodeID+1).totalDistance;
                if costChange < 0
                    
                    Path = Update_from_Node(tempPath,randomNodeID+2,costChange,timeChange,distanceChange);
                    
                    rndNodePathTreeID = randomNode.ID;
                    
                    if randomNode.TreeNo == 1
                        pathTree1(rndNodePathTreeID).x = q_rand(1); 
                        pathTree1(rndNodePathTreeID).y = q_rand(2);
                        pathTree1(rndNodePathTreeID).psi = heading(pathTree1(randomNode.parent),pathTree1(rndNodePathTreeID));
                        
                        pathTree1 = childrenNodesUpdate(pathTree1,Tree1,rndNodePathTreeID,costChange,distanceChange,timeChange);
                    else
                        pathTree2(rndNodePathTreeID).x = q_rand(1); 
                        pathTree2(rndNodePathTreeID).y = q_rand(2);
%                         pathTree2(rndNodePathTreeID).psi = heading(pathTree2(randomNode.parent),pathTree2(rndNodePathTreeID));
                        
                        pathTree2 = childrenNodesUpdate(pathTree2,Tree2,rndNodePathTreeID,costChange,distanceChange,timeChange);
                    end
                    
% %                                                 l =  plot([Path(:).x]',[Path(:).y]','m','LineWidth', 2);
% %                     
% %                     
% %                     
% %                     %                             frm = getframe;
% %                     %                             img = frame2im(frm);
% %                     %                             writeVideo(writerObj, img);
% %                                                 delete(l);
                    
                else
                    
                    tempPath = Path;
                end
            
            end
        
        end
    
    end
    

    
% %     l =  plot3([Path(:).x]',[Path(:).y]',Vmax*ones(length(Path),1),'m','LineWidth', 2.25,'DisplayName','Smoothed Path');
% %     hold on
%     legend([str 'smooth'],'location','north','orientation','horizontal');   
% %     close(writerObj);
    
    FinalSmoothedPath = Path;
    smoothTotal_distance = Path(end).totalDistance;
    smooth_REL = Path(end).CumulativeCost;

end