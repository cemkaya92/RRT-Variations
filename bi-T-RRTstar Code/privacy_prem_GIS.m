% This file is created by U. Cem Kaya - 2018
function privacy_PREM = privacy_prem_GIS(buildingData,Grid_x,Grid_y,X,Y) 

N=length(buildingData);
% figHandle = figure;
privacy_PREM = zeros(Grid_y,Grid_x);  % Initialize the PREM evaluation

BoundingBox_x = zeros(N,2);
BoundingBox_y = zeros(N,2);
center = zeros(N,2);
corners_X = zeros(N,5);
corners_Y = zeros(N,5);

total_building_area = 0;
dArea = (X(1,2)-X(1,1))*(Y(2,1)-Y(1,1)); % small grid areas

for k=1:N   % find the centroid of each building
%    % posX=building(k).BoundingBox(1,1)*0.3048;  % x coordinates in meters
%    % posY=building(k).BoundingBox*0.3048;  % y coordinates in meters
%     building(k).latRange=[buildingData(k).BoundingBox(1,1) , buildingData(k).BoundingBox(2,1)]*0.3048/1000; % in km
%     building(k).longRange=[buildingData(k).BoundingBox(1,2) , buildingData(k).BoundingBox(2,2)]*0.3048/1000;
 
%     building(k).center = [sum(building(k).latRange) , sum(building(k).longRange)]/2;

    BoundingBox_x(k,:) = [buildingData(k).BoundingBox(1,1) , buildingData(k).BoundingBox(2,1)]/1000; % in km
    BoundingBox_y(k,:) = [buildingData(k).BoundingBox(1,2) , buildingData(k).BoundingBox(2,2)]/1000;
    
% %     center(k,:) =[sum(BoundingBox_x(k,:)) , sum(BoundingBox_y(k,:))]/2;
% %     
% %     corners_X(k,:) = [buildingData(k).BoundingBox(1,1) , buildingData(k).BoundingBox(2,1),...
% %                     buildingData(k).BoundingBox(2,1) , buildingData(k).BoundingBox(1,1),buildingData(k).BoundingBox(1,1)]/1000;
% %                 
% %     corners_Y(k,:) = [buildingData(k).BoundingBox(1,2) , buildingData(k).BoundingBox(1,2),...
% %                     buildingData(k).BoundingBox(2,2) , buildingData(k).BoundingBox(2,2),buildingData(k).BoundingBox(1,2)]/1000;
% %     
    idx = inpolygon(X,Y,BoundingBox_x(k,:)',BoundingBox_y(k,:)') ;
% %     figure(figHandle)
% %     plot(corners_X(k,:)',corners_Y(k,:)')
    building_covered_area = ((BoundingBox_x(k,2)-BoundingBox_x(k,1))*(BoundingBox_y(k,2)-BoundingBox_y(k,1)));
    
    privacy_PREM(idx) = privacy_PREM(idx) + dArea;
    total_building_area = total_building_area + building_covered_area;
end

privacy_PREM = privacy_PREM/total_building_area;




end
