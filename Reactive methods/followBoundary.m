function [R,getMode,activeLayer,layer,noLayer,removefromSTM] = followBoundary(layer,activeLayer,...
    prevEnvironment,Rt,tenacity,getMode,prevRt,sensorData,scan,angleToGoal,...
    K,outer_loop,cur_x,cur_y)
% This function returns the angular region in which the robot should move
% when following a boundary
removefromSTM = [];
% Change the direction of searches depending on tenacity
if tenacity == 1
    searchA = 1; %clockwise
    searchB = 2; %counterclockwise
else
    searchA = 2; %counterclockwise
    searchB = 1; %clockwise
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This section creates a new layer if old one is fully banned:
% If all regions are blocked, then a new activeLayer is added and is made active,
% denoted by the variable activeLayer. Only the last one and the current activeLayer 
% are ever stored.
newLayer = 0;
% Check if sectors exist (needed for first run of programme)
% store the environment from last step as (1).
if isfield(layer(activeLayer),'environment') && ...
        isfield(layer(activeLayer).environment,'sector')
%     prevEnvironment = layer(activeLayer).environment;
%     prevRt = layer(activeLayer).environment.Rt;
    if all(strcmp(layer(activeLayer).environment.sector,'blocked'))
        % create a new activeLayer and make it active
        activeLayer = activeLayer+1;
        % remember at which step this occured
        newLayer = 1;
        % Need to re-initialise the obstacle_storage structure for new
        % layer
        layer(activeLayer).obstacle_storage = [];
        layer(activeLayer).obstacle_storage.sector = [];
        layer(activeLayer).obstacle_storage.angle = [];
        layer(activeLayer).obstacle_storage.distance = [];
        layer(activeLayer).obstacle_storage.centre = [];
        % Create new layer:
        [layer,sensorData,activeLayer,...
    prevEnvironment,Rt,prevRt]=createSectorEnvironment_EG_v2(layer,activeLayer,scan,...
    K,angleToGoal,cur_x,cur_y,outer_loop);       
    else
        newLayer = 0;
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Escape Gap uses so-called MAINCND condition to decide which angular
% region to select, defined as follows:
if newLayer==0 && strcmp(prevEnvironment.sector(prevRt),'blocked') && ...
        strcmp(layer(activeLayer).environment.sector(Rt),'allowed')
    MAINCND = true;
else
    MAINCND = false;
end


if MAINCND
    % If the currently active layer is not the first one, delete it and
    % move to the previous one
    if activeLayer ~= 1
        % before clearing the layer, need to remember the Rt (otherwise it
        % will use the Rt from when the layer got full)
        layer(activeLayer-1).environment.Rt = Rt;
        layer(activeLayer) = [];
        activeLayer = activeLayer - 1;
        layer(activeLayer).environment.sector(Rt)='allowed';
        removefromSTM(end+1) = Rt;
    else
       layer(activeLayer).environment = []; 
       noLayer = 1;
       layer(activeLayer).obstacle_storage.sector = [];
       layer(activeLayer).obstacle_storage.angle = [];
       layer(activeLayer).obstacle_storage.distance = [];
       layer(activeLayer).obstacle_storage.centre = [];
       getMode = 1;
    end
    R=Rt;   
    % and obtain new sensorData:
    [~,sensorData]=createSectorEnvironment_EG_v2(layer,activeLayer,scan,...
    K,angleToGoal,cur_x,cur_y,outer_loop);
elseif ~MAINCND && strcmp(layer(activeLayer).environment.sector(Rt),'blocked') 
    % mark the Rt sector as allowed in order to search its neighbours
    layer(activeLayer).environment.sector(Rt) = 'allowed';
    % use function to find the next allowed region (based on tenacity)
    R = calculateR(Rt,layer(activeLayer),'allowed',searchA,'EG');
    % set Rt sector back to blocked
    layer(activeLayer).environment.sector(Rt) = 'blocked';
else
    % otherwise find the closest to Rt blocked region (based on tenacity)
    if strcmp(layer(activeLayer).environment.sector(Rt),'allowed') 
        layer(activeLayer).environment.sector(Rt)='blocked';
        R_blck = calculateR(Rt,layer(activeLayer),'blocked',searchB,'EG');
        layer(activeLayer).environment.sector(Rt)='allowed';
    else
        R_blck = calculateR(Rt,layer(activeLayer),'blocked',searchB,'EG');
    end
    
    % and then pick the allowed region closest to it (based on tenacity
    % again)
    layer(activeLayer).environment.sector(R_blck) = 'allowed'; 
    R = calculateR(R_blck,layer(activeLayer),'allowed',searchA,'EG');
    layer(activeLayer).environment.sector(R_blck) = 'blocked';    
end

if ~exist('noLayer','var')
    noLayer = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data Removal Section:
if noLayer == 0
    %
    % Label R region as blocked for now:
     layer(activeLayer).environment.sector(R)='blocked';

    % Find the closest to R banned region 
    Rcheck = calculateR(R,layer(activeLayer),'blocked',searchB,'EG');
    % Re-label R as allowed again
    layer(activeLayer).environment.sector(R)='allowed';
    % Check if it's consistent with the current sensor data:
    [cond]=isInconsistent_EG(Rcheck,layer(activeLayer),sensorData(activeLayer));

    if cond
        layer(activeLayer).environment.sector(Rcheck)='allowed';
        % Remove Rcheck from STM
        removefromSTM(end+1) = Rcheck;
        R = Rcheck;
    end
else
   Rcheck = NaN; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end