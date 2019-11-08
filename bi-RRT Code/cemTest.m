% This file is created by U. Cem Kaya - 2018

clear all, clc, close all
mkdir figs
dir = pwd;
pathFigs = [dir '\figs'];
tic
      
    rng(10)
    
    p_quasi = [];%haltonset(2,'Skip',1e3,'Leap',1e2); % quasi random numbers
    
    %% Define Parameters
    start = [664.4235 3625.3280];
    finish = [664.3190 3625.5546];

    n = 1.5; % scaling of the velocity w/o changing grid size
    V_des = n*60/3600; %km/sec
    solver_delta_t = 1; 
    
    distThresh = 0.1*V_des*solver_delta_t;
    stepSize = V_des*solver_delta_t;
    iterMax = 10000; %maximum number of iterations

    goalProb = 0.01;
    goalProbTree2 = 0.02;
    randNodeExtProb = 0.02;
    delayOption = 1; %1-no pause, 2 - 0.15 s pause, 3 - 0.2 s pause
    
    grid_size = 2.5*stepSize/(4*n);
    
    %% Kinematic/Dynamic Constraints
    Constraints.psi_dot = (60)*pi/180;
    Constraints.a_max = 5; % m/s2
    Constraints.V_max = 50; % m/s
    Constraints.delta_t = [0.8*solver_delta_t 1.2*solver_delta_t]; % dt_min and dt_max to extend
    
    plotBranches = 0;% true/false

    RecordVideo = 1;

if isfile('Scenario11.mat')
    load('Scenario11.mat');
else
    [Building_PREM_value,Traffic_PREM_value,Populations,mapSize,dX,dY,N,X,Y,...
     BuildingCenters,corners_X,corners_Y,Grid_x,Grid_y,buildingData,utmstruct]...
        = building_density_GIS(grid_size);
    
%     privacy_PREM = privacy_prem_GIS(buildingData,Grid_x,Grid_y,X,Y);
privacy_PREM = [];
    
    [startGPS_lat,startGPS_lon] = minvtran(utmstruct,start(1)*1000,start(2)*1000);    
    [finish_lat,finish_lon] = minvtran(utmstruct,finish(1)*1000,finish(2)*1000);
    
    start_GPS = [startGPS_lat,startGPS_lon];
    finish_GPS = [finish_lat,finish_lon];
% save prem variables into .mat file
    save('Scenario11','Building_PREM_value','Traffic_PREM_value','privacy_PREM',...
        'mapSize','dX','dY','X','Y','BuildingCenters','buildingData','utmstruct',...
        'startGPS_lat','startGPS_lon','finish_lat','finish_lon');
end
clear buildingData;

riskWeighting = [1 ;0.0];

PREM_value = Building_PREM_value*riskWeighting(1) + Traffic_PREM_value*riskWeighting(2);

PREM.layer1 = PREM_value;
PREM.layer2 = privacy_PREM;
PREM.weights = [1 0]; 
Population = 1;

figHandle = plots(mapSize,BuildingCenters,Building_PREM_value,PREM_value,start,finish,X,Y);


PURM.theta = (pi/180)*[0 -30 0 30]';
PURM.reachableRadius = [(6)*stepSize (4)*stepSize; (4.5)*stepSize 2.5*stepSize; (4)*stepSize 2*stepSize; (4.5)*stepSize 2.5*stepSize]/(2*2*n);
PURM.failureRate = [10^(-5) 10^(-4) 10^(-3) 10^(-4)]'/3600;
    
% % %     PURM_plot(PURM,dX,dY,pathFigs);
    
RiskThreshold.Cumulative = (inf);
RiskThreshold.ExposureRate = (inf);
RiskThreshold.TotalLength = inf;

d = max(min(PURM.reachableRadius)); % keep away from walls
ResMapSize = [mapSize(1,1)+d mapSize(1,2)+d;
mapSize(2,1)-d mapSize(2,2)-d];

FirstPaths = {};
FinalPaths = {};

    for run = 1:1
        
%         obstacles = polygonalObstacles(mapSize,figHandle);
        obstacles = [];
        
    [MultiplePaths,finalPath,FinalSmoothedPath,riskExposureLevels]...
        = mainFunc(mapSize, start, finish, distThresh,stepSize, iterMax, goalProb, delayOption, figHandle,...
        V_des,PREM,dX,dY,PURM,X,Y,Population,goalProbTree2,RiskThreshold,obstacles,ResMapSize,Constraints,...
        plotBranches,RecordVideo,randNodeExtProb,p_quasi,pathFigs);
    
    
% %    h = AllPlots(figHandle,MultiplePaths,finalPath,PREM,X,Y,mapSize,BuildingCenters,corners_X,corners_Y,start,finish);
% %    
% %     [FinalSmoothedPath,smoothTotal_distance,smooth_REL] = LocalBiasingSmoothing(finalPath,...
% %     PREM,ResMapSize,dX,dY,PURM,X,Y,Population,V_des,figHandle,RiskThreshold,obstacles,...
% %     10000);
   
    end

   
    toc
  
    
    % save path solutions into .mat file  
    save('SavedScenario','MultiplePaths','finalPath','FinalSmoothedPath',...
        'riskExposureLevels','mapSize','start','finish','stepSize','PURM',...
        'PREM','V_des','obstacles','start_GPS','finish_GPS');

