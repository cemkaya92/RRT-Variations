% This file is created by U. Cem Kaya - 2018
function [Building_PREM_value,Traffic_PREM_value,Populations,mapSize,dX,dY,...
    N,X,Y,BuildingCenters,corners_X,corners_Y,Grid_x,Grid_y,buildingData,...
    utmstruct] = building_density_GIS(grid_size) 

Populations = [];

buildingData=shaperead('map1new.shp','UseGeoCoords', true);
N=length(buildingData);

Populations(end+1) = N;

% grid_size=100/1000; %in km
% grid_size=10/1000; %in km
r = (grid_size/2)*sqrt(2);

% using corner of the buildings, find minimum rectangle as mapSize
mapSize_GPS = [min([buildingData(:).Lat]) min([buildingData(:).Lon]);
            max([buildingData(:).Lat]) max([buildingData(:).Lon])];% GPS

% find the UTM zone using the center of the map
UTMzone = utmzone(0.5*(mapSize_GPS(1,1)+mapSize_GPS(2,1)),0.5*(mapSize_GPS(1,2)+mapSize_GPS(2,2)));

% assign UTM zone as a coordinate frame
utmstruct = defaultm('utm'); 
utmstruct.zone = UTMzone;  
utmstruct.geoid = wgs84Ellipsoid;
utmstruct = defaultm(utmstruct);

% mapSize in meters
[mapSize(:,1),mapSize(:,2)] = mfwdtran(utmstruct,mapSize_GPS(:,1),mapSize_GPS(:,2));

mapSize = mapSize/1000; %in km

% add an empty frame around the map
mapSize(1,1) = mapSize(1,1)-0.1*grid_size; mapSize(2,1) = mapSize(2,1)+0.1*grid_size;
mapSize(1,2) = mapSize(1,2)-0.1*grid_size; mapSize(2,2) = mapSize(2,2)+0.1*grid_size;

lat_min=mapSize(1,1);
lon_min=mapSize(1,2);

lat_grid=lat_min:grid_size:mapSize(2,1); % in Km
lon_grid=lon_min:grid_size:mapSize(2,2);

lat_max=lat_grid(end);
lon_max=lon_grid(end);

LongGridNumber = length(lon_grid)-1;
LatGridNumber = length(lat_grid)-1;

Grid_Matrix = zeros( LongGridNumber , LatGridNumber ); 
% % Grid_BuildingCenter_X = zeros( length(mapSize(1,2):grid_size:mapSize(2,2)) , length(mapSize(1,1):grid_size:mapSize(2,1)) ); 
% % Grid_BuildingCenter_Y = zeros( length(mapSize(1,2):grid_size:mapSize(2,2)) , length(mapSize(1,1):grid_size:mapSize(2,1)) ); 

%% calculate area limits and generate grid
% memory allocation for speed
% building = struct('center',zeros(N,2),'latRange',zeros(N,2),'longRange',zeros(N,2),'gridID',zeros(N,2),'gridNumber',zeros(N,1));
latRange = zeros(N,2);
longRange = zeros(N,2);
center = zeros(N,2);
% gridID = zeros(N,2);
% gridNumber = zeros(N,1);
corners_X = zeros(N,4); % Bounding box is taken as corners of a building
corners_Y = zeros(N,4);



% emptyID = [0,0];

% figure(figHandle); %% Plot Center of Buildings
  % Plot Grid 

for k=1:N   % find the centroid of each building
%     GridNumber = 1;
    id_found = 0;
%    % posX=building(k).BoundingBox(1,1)*0.3048;  % x coordinates in meters
%    % posY=building(k).BoundingBox*0.3048;  % y coordinates in meters
%     building(k).latRange=[buildingData(k).BoundingBox(1,1) , buildingData(k).BoundingBox(2,1)]*0.3048/1000; % in km
%     building(k).longRange=[buildingData(k).BoundingBox(1,2) , buildingData(k).BoundingBox(2,2)]*0.3048/1000;
 
%     building(k).center = [sum(building(k).latRange) , sum(building(k).longRange)]/2;

    longRange(k,:) = [buildingData(k).BoundingBox(1,1) , buildingData(k).BoundingBox(2,1)]; % in GPS
    latRange(k,:) = [buildingData(k).BoundingBox(1,2) , buildingData(k).BoundingBox(2,2)];
    
    [x_range,y_range] = mfwdtran(utmstruct,latRange(k,:)',longRange(k,:)'); %in meters
         
    center(k,:) =[sum(x_range) , sum(y_range)]/(2000); % in km
    
    corners_X(k,:) = [buildingData(k).BoundingBox(1,1) , buildingData(k).BoundingBox(2,1),...
                    buildingData(k).BoundingBox(2,1) , buildingData(k).BoundingBox(1,1)]/1000;
                
    corners_Y(k,:) = [buildingData(k).BoundingBox(1,2) , buildingData(k).BoundingBox(2,2),...
                    buildingData(k).BoundingBox(2,2) , buildingData(k).BoundingBox(1,2)]/1000;
    
    for a=1:LatGridNumber  %horizontal sweep
        for b=1:LongGridNumber  %vertical sweep
            if (center(k,1)>=lat_grid(a) && center(k,1)<lat_grid(a+1))...
                    && (center(k,2)>=lon_grid(b) && center(k,2)<lon_grid(b+1))
                              
%                  gridID(k,:) = [ b,a];
                 Grid_Matrix(b, a) = Grid_Matrix( b, a) + 1;
%                  Grid_BuildingCenter_X(length(lon_grid) - b, a) =  [ center(k,1);
%                  Grid_BuildingCenter_Y(length(lon_grid) - b, a) =  [ center(k,2);
%                  gridNumber(k,:) = GridNumber;
                 
%                 building(k).gridID = [a,b];
%                 building(k).gridNumber = GridNumber;
% % %                 rectangle('Position',[corners_X(k,1) corners_Y(k,1)...
% % %                          (corners_X(k,3)-corners_X(k,1)) (corners_Y(k,3)-corners_Y(k,1))] );
% % %                 
% % %                 plot(center(k,1),center(k,2),'.-');
% % %                 hold on
% %                 
                id_found = 1;
                break;
            end
%          GridNumber = GridNumber + 1;
        end
        
        if (id_found == 1)          
            break;
        end
        
    end
end

Grid_Matrix = flipud(Grid_Matrix);
axis([lat_min lat_max lon_min lon_max]);

grid on
BuildingCenters = center;


%% Find buildings in each Grid and Calculate Mean and Variance
%  OccupiedGridNumbers = unique(gridNumber);
[I,J] = find(Grid_Matrix);    %%OccupiedGridNumbers = unique(gridNumber);
 OccupiedGridNumbers = length(I);
 
mu_x = zeros(OccupiedGridNumbers,1);
mu_y = zeros(OccupiedGridNumbers,1);
% variance_X = zeros(OccupiedGridNumbers,1);
% variance_Y = zeros(OccupiedGridNumbers,1);
sigma_x = zeros(OccupiedGridNumbers,1);
sigma_y = zeros(OccupiedGridNumbers,1);
buildingNumberInCurrentGrid = zeros(OccupiedGridNumbers,1);

% figure(1)

%     maxNumberBuildingInGrid = max(max(Grid_Matrix));
 for j = 1:(OccupiedGridNumbers)
%     number_of_occurance = sum(OccupiedGridNumbers(j) == OccupiedGridNumbers(:));
%     buildingNumberInCurrentGrid(j) = Grid_Matrix(I(j),J(j))/max(max(Grid_Matrix(I(j),J(j)))); % Normalized by total number of buildings in the current grid
    
%     buildingNumberInCurrentGrid(j) = Grid_Matrix(I(j),J(j))/maxNumberBuildingInGrid;
%     buildingNumberInCurrentGrid(j) = Grid_Matrix(I(j),J(j));
    buildingNumberInCurrentGrid(j) = Grid_Matrix(I(j),J(j))/N;    

    
    CurrentGrid_X = lat_min + J(j)*grid_size - grid_size/2 ;
    CurrentGrid_Y = lon_max - I(j)*grid_size + grid_size/2 ;
    
%     mu_x(j) = CurrentGrid_X;
%     mu_y(j) = CurrentGrid_Y;

    mu_x(j) = CurrentGrid_X;
    mu_y(j) = CurrentGrid_Y;
    
%     variance_X(j) = var(buildingCornersInCurrentGrid_X(:));
%     variance_Y(j) = var(buildingCornersInCurrentGrid_Y(:));
%     
    sigma_x(j) = r/0.9058; % 
    sigma_y(j) = r/0.9058 ;
    

 end
 
      %% Plot PREM
BuildngDensity = buildingNumberInCurrentGrid;

% mu =    [ mu_x , mu_y ];   
% sigma = [ sigma_x , sigma_y ];     

% threshold =  (10^-4);
%threshold =  0.25; %in km

prem = [];


    % Create the list of PDF structures for the PREM
    for ii = length(mu_x):-1:1
        pdfs(ii) = newPdf([mu_x(ii) mu_y(ii)],[sigma_x(ii)^2 0; 0 sigma_y(ii)^2]);
    end
    prem.pdfs = pdfs;
%     prem.threshold = threshold;

%      PREMplot(prem,mapSize,mapSize,figHandle);
%      [PREM_value,dX,dY,X_Grid,Y_Grid,X,Y] = PREMplot(density,prem,[lat_min lat_max],[lon_min lon_max],length(lat_grid)-1,length(lon_grid)-1);
% [PREM_value,dX,dY,X_Grid,Y_Grid,X,Y] = PREMplot(density,prem,[lat_min lat_max],[lon_min lon_max],200,200);
Grid_x = 5*(length(lat_grid)-1); Grid_y = 5*(length(lon_grid)-1); % I have -1 here since length of lon/lat_grid are 1 more than number of grid between range 
[Building_PREM_value,dX,dY,X,Y] = PREMplot(BuildngDensity,prem,[lat_min lat_max],[lon_min lon_max],Grid_x,Grid_y,N);


% % load('traffic.mat');
% % 
% % N = length(trafficX);
% % 
% % Populations(end+1) = N;
% % 
% % Grid_MatrixTraffic = zeros( LongGridNumber , LatGridNumber ); 
% % 
% % for k=1:N  % find the centroid of each building
% % 
% %     id_found = 0;
% %     
% %     center(k,:) =[trafficX(k) , trafficY(k)];
% %     
% %     for a=1:length(lat_grid)-1  %horizontal sweep
% %         for b=1:length(lon_grid)-1  %vertical sweep
% %             if (center(k,1)>=lat_grid(a) && center(k,1)<lat_grid(a+1))...
% %                     && (center(k,2)>=lon_grid(b) && center(k,2)<lon_grid(b+1))
% %                               
% %                  
% %                  Grid_MatrixTraffic(b, a) = Grid_MatrixTraffic( b, a) + 1;
% %           
% %                 id_found = 1;
% %                 break;
% %             end
% % 
% %         end
% %         
% %         if (id_found == 1)          
% %             break;
% %         end
% %         
% %     end
% % end
% % 
% % Grid_MatrixTraffic = flipud(Grid_MatrixTraffic);
% % 
% % [I,J] = find(Grid_MatrixTraffic);    %%OccupiedGridNumbers = unique(gridNumber);
% %  OccupiedGridNumbers = length(I);
% %  
% % mu_x = zeros(OccupiedGridNumbers,1);
% % mu_y = zeros(OccupiedGridNumbers,1);
% % 
% % sigma_x = zeros(OccupiedGridNumbers,1);
% % sigma_y = zeros(OccupiedGridNumbers,1);
% % CarNumberInCurrentGrid = zeros(OccupiedGridNumbers,1);
% % 
% %  for j = 1:(OccupiedGridNumbers)
% %      
% %     CarNumberInCurrentGrid(j) = Grid_MatrixTraffic(I(j),J(j))/N;    
% % 
% %     
% %     CurrentGrid_X = lat_min + J(j)*grid_size - grid_size/2 ;
% %     CurrentGrid_Y = lon_max - I(j)*grid_size + grid_size/2 ;
% %     
% % 
% % 
% %     mu_x(j) = CurrentGrid_X;
% %     mu_y(j) = CurrentGrid_Y;
% %     
% % 
% %     sigma_x(j) = r/0.9058; % 
% %     sigma_y(j) = r/0.9058 ;
% %     
% % 
% % 
% %  end
% %  
% %       %% Plot PREM
% % TrafficDensity = CarNumberInCurrentGrid;
% % 
% % % mu =    [ mu_x , mu_y ];   
% % % sigma = [ sigma_x , sigma_y ];     
% % 
% % 
% % prem = [];
% % 
% % 
% %     % Create the list of PDF structures for the PREM
% %     for ii = 1:length(mu_x)
% %         pdfs2(ii) = newPdf([mu_x(ii) mu_y(ii)],[sigma_x(ii)^2 0; 0 sigma_y(ii)^2]);
% %     end
% %     prem.pdfs = pdfs2;
% % 
% % [Traffic_PREM_value,dX,dY,X,Y] = PREMplot(TrafficDensity,prem,[lat_min lat_max],[lon_min lon_max],Grid_x,Grid_y,N);

Traffic_PREM_value = 0;
end
