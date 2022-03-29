function [scenario,assessment,laneInfo] = helperGetLaneFollowingScenario(roadType,stopTime, Ts)
%helperGetLaneFollowingScenario gets the driving scenario synchronized with
%HighwayLaneFollowingTestBench.slx
%
%   'roadType'      Type of Sim 3D road scene. Valid values:
%                   "Straight road" synchronize with "Straight road"
%                   Sim 3D scene
%                   "Curved road segment" synchronize with segment of
%                   "Curved road" Sim 3D scene
%                   "RR Highway road" that has the scene designed in 
%                   RoadRunner
%
%   'scenario'      Driving scenario with vehicles matching with
%                   HighwayLaneFollowingTestBench.slx
%
%   'assessment'    Assessment structure used by
%                   HighwayLaneFollowingTestBench.slx
%
%   'laneInfo'      Lane Information of the scene. It is a structure that
%                   contains lane width, lane direction and lane centers
%                   which are helpful to specify vehicle trajectories.
%

%   This is a helper function for example purposes and
%   may be removed or modified in the future.

%   Copyright 2019-2021 The MathWorks, Inc.

%% Construct driving scenario
scenario = drivingScenario;
scenario.SampleTime = Ts;
scenario.StopTime = stopTime;
%% Add road to scenario
switch roadType
    case "Straight road"
        laneInfo = addStraightRoad(scenario);
        defaultVehiclePosition = [-122 50 0];
    case "Curved road segment"
        laneInfo = addCurvedRoadSegment(scenario);
        defaultVehiclePosition = [0 0 0];
    case "RR Highway road"                     
        laneInfo = addOpenDriveRoad(scenario);
        defaultVehiclePosition = [0 0 0];
    otherwise
        error("Unsupported road type. Road type must be 'Straight road' or 'Curved road segment' or 'RR Highway Road'");
end

%% Add vehicles to scenario
% - Vehicle profiles must match vehicles in Simulink model
% - First vehicle added is ego (ActorID = 1)
helperAddVehicle(scenario, "Sedan",                 defaultVehiclePosition);
helperAddVehicle(scenario, "Sedan",                 defaultVehiclePosition);
helperAddVehicle(scenario, "Muscle car",            defaultVehiclePosition);
helperAddVehicle(scenario, "Hatchback",             defaultVehiclePosition);
helperAddVehicle(scenario, "Small pickup truck",    defaultVehiclePosition);
helperAddVehicle(scenario, "Sport utility vehicle", defaultVehiclePosition);

%% Default assessments
assessment.TimeGap = 0.8;
assessment.LongitudinalAccelMin = -3;
assessment.LongitudinalAccelMax = 3;
assessment.LateralDeviationMax = 0.45;
% Lane metrics assessments
assessment.LaneDetector.Precision = 90;
assessment.LaneDetector.Sensitivity = 90;

end

function laneInfo = addStraightRoad(scenario)
% Add road representing "Straight road" Sim 3D scene
rc = load('laneFollowingRoadCenters','roadCentersStraightRoad');
roadCenters = rc.roadCentersStraightRoad;

marking = [...
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Unmarked')];
laneWidth = [1.15 3.85 4.05 4.05 3.85 1.15];
laneSpecification = lanespec([3 3], 'Width', laneWidth, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

[n,~] = size(roadCenters);

laneCenters = {zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3)};
numLanes = numel(laneSpecification.Width);
offset = estimateRoadCenterOffset(laneWidth);
minLaneWidth = 2;
laneInfo = struct();
% Shifting of road centers to get lane centers
for j = 1:numLanes
    for i = 1:n
        laneCenters{j}(i,1) = roadCenters(i,1) + 0;
        laneCenters{j}(i,2) = roadCenters(i,2) + offset{j};
        laneCenters{j}(i,3) = roadCenters(i,3) + 0;
    end
    laneInfo(j).LaneWidth     = laneSpecification.Width(j);
    
    if laneSpecification.Width(j) <= minLaneWidth
        laneInfo(j).LaneDirection = 0;
        laneInfo(j).LaneCenters   = laneCenters{j};
    else
        % Lanes with indices 2,3 are considered in positive direction
        if j/(numLanes/2) > 1
            laneInfo(j).LaneDirection = 1;
            laneInfo(j).LaneCenters   = laneCenters{j};
        else
            % Lanes with indices 4,5 are considered in negative direction
            laneInfo(j).LaneDirection = -1;
            laneInfo(j).LaneCenters   = flip(laneCenters{j});
        end
    end
end
end

function laneInfo = addCurvedRoadSegment(scenario)
% Add road representing a segment of the "Curved road" Sim 3D scene
rc = load('laneFollowingRoadCenters','roadCentersCurvedRoadSegment');
roadCenters = rc.roadCentersCurvedRoadSegment;

marking = [...
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Unmarked')];
laneWidth = [1.15 3.85 4.05 4.05 3.85 1.15];
laneSpecification = lanespec(6, 'Width', laneWidth, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

laneInfo = estimateLaneCenters(roadCenters, laneWidth);

end

function laneInfo = estimateLaneCenters(roadCenters, laneWidth)
% estimateLaneCenters computes laneCenters from roadCenters based on
% lane width for all the lanes in the scenario.
% RoadCenterDirection is defined as the positive increment of index in
% roadCenters array.
% laneInfo   : Structure that has below fields 
%              LaneWidth : Width of the lane.
%              LaneDirection: Direction of the lane. 
%              If the lane is in RoadCenterDirection, it is 1.
%              If the lane is opposite to the RoadCenterDirection, it is
%              -1.
%              If the lane is border lane, it is 0.
%              LaneCenters: Center of the lanes. Lane centers are flipped
%              in case of  LaneDirection opposite to the RoadCenterDirection
%           
% laneInfo{1}: Left border  lane.
% laneInfo{2}: Leftmost lane from road center in RoadCenterDirection.
% laneInfo{3}: Immediate left lane from road center in RoadCenterDirection.
% laneInfo{4}: Immediate right lane from road center in RoadCenterDirection.
% laneInfo{5}: Rightmost lane from road center in RoadCenterDirection.
% laneInfo{6}: Right border lane.

% Get number of lanes
numLanes = size(laneWidth, 2);
% Get road center offsets for all lanes
offset = estimateRoadCenterOffset(laneWidth);
% Initialize lane centers array
[n,~] = size(roadCenters);
laneCenters = repmat({zeros(n, 3)}, 1, numLanes);
minLaneWidth = 2;
laneInfo = struct();
% Compute lane centers for all lanes by using road center offsets 
for j = 1:numLanes
    k = 1;
    for i = 1:n-1
        % Get distance between current point and next waypoint
        span = hypot(roadCenters(i+1,1)-roadCenters(i,1),roadCenters(i+1,2)-roadCenters(i,2));
        
        % Reject the waypoint if the distance is less than 0.5 meters
        if (span < 0.5)
            continue;
        end
        theta = asin((roadCenters(i+1,2)-roadCenters(i,2))/span);
        
        if (roadCenters(i+1,1)-roadCenters(i,1)) > 0
            delX = -offset{j} * sin(theta);
            delY = offset{j} * cos(theta);
        else
            delX = -offset{j} * sin(theta);
            delY = -offset{j} * cos(theta);
        end
        
        laneCenters{j}(k,1) = roadCenters(i,1) + delX;
        laneCenters{j}(k,2) = roadCenters(i,2) + delY;
        laneCenters{j}(k,3) = 0;
        
        k = k+1;        
    end

    laneCenters{j}(k,1) = roadCenters(n,1) + delX;
    laneCenters{j}(k,2) = roadCenters(n,2) + delY;
    laneCenters{j}(k,3) = 0;
    
    % Find lane direction and accordingly flip lane centers
    if j/(numLanes/2)>1
        laneInfo(j).LaneWidth     = laneWidth(j);
        if laneWidth(j) > minLaneWidth
            laneInfo(j).LaneDirection = 1;
        else
            laneInfo(j).LaneDirection = 0;
        end
        laneInfo(j).LaneCenters   = laneCenters{j}(2:end-1,:);
    else
        laneInfo(j).LaneWidth     = laneWidth(j);
        if laneWidth(j) > minLaneWidth
            laneInfo(j).LaneDirection = -1;
            laneInfo(j).LaneCenters   = flip(laneCenters{j}(2:end-1,:));
        else
            laneInfo(j).LaneDirection = 0;
            laneInfo(j).LaneCenters   = laneCenters{j}(2:end-1,:);
        end
    end
end
end

function offset = estimateRoadCenterOffset(laneWidth)
% estimateRoadCenterOffset takes laneWidth as input and gives offsets for
% each lane from road center. 
% 'offset' means distance from the road center.
% Output : offset as a cell array.

% Compute number of lanes
laneWidthSize = size(laneWidth);
numLanes = laneWidthSize(2);

% Lane width less than minLaneWith is not considered as lane for offset
% calculation
minLaneWidth = 2; % Distance in meter
offset = cell(1,numLanes);
i = 1;

% If number of Lanes are even
    if mod(numLanes,2) == 0
        leftWidths = laneWidth(1:numLanes/2);
        rightWidths = laneWidth((numLanes/2)+1:end);
    
        % offset is calculated based on the Lane width of each lane
        for j = 1:length(leftWidths)
            if j < length(leftWidths)
                % To get the offset of current lane from road center, add
                % half the width of current lane and widths of all other
                % lanes that are between the current lane and the road
                % center
                offset{i} = (leftWidths(j)/2) + sum(leftWidths(j+1:end));
                i = i+1;
            else
                offset{i} = leftWidths(j)/2;
                i = i+1;
            end
        end
    
        % Negative sign is used based on convention that Lane offset towards
        % left of road centers are positive and towards right are negative
        for j = 1:length(rightWidths)
            if j == 1
                offset{i} = -rightWidths(j)/2;
                i = i+1;
            else
                % To get the offset of current lane from road center, add
                % half the width of current lane and widths of all other
                % lanes that are between the current lane and the road
                % center
                offset{i} = -((rightWidths(j)/2) + sum(rightWidths(1:j-1)));
                i = i+1;
            end
        end
    % If number of Lanes are odd
    else
        leftWidths = laneWidth(1:floor(numLanes/2));
        rightWidths = laneWidth(ceil(numLanes/2)+1:end);
        centerLaneWidth = laneWidth(ceil(numLanes/2));
        for j = 1:length(leftWidths)
            if leftWidths(j) > minLaneWidth
                if j < length(leftWidths)
                    % To get the offset of current lane from road center,
                    % add half the width of current lane and widths of all
                    % other lanes that are between the current lane and the
                    % road center
                    offset{i} = (centerLaneWidth/2) + (leftWidths(j)/2) + sum(leftWidths(j+1:end));
                    i = i+1;
                else
                    offset{i} = leftWidths(j)/2 + (centerLaneWidth/2);
                    i = i+1;
                end
            end
        end
        offset{ceil(numLanes/2)} = 0;
        i = i+1;
        for j = 1:length(rightWidths)
            if rightWidths(j) > minLaneWidth
                if j == 1
                    offset{i} = -((rightWidths(j)/2) + (centerLaneWidth/2));
                    i = i+1;
                else
                    % To get the offset of current lane from road center,
                    % add half the width of current lane and widths of all
                    % other lanes that are between the current lane and the
                    % road center
                    offset{i} = -((centerLaneWidth/2) + (rightWidths(j)/2) + sum(rightWidths(1:j-1)));
                    i = i+1;
                end
            end
        end
    end
end

function laneInfo = addOpenDriveRoad(scenario)
%addOpenDriveRoad updates roadnetwork to the scenario using
%"RRHighway.xodr".

% Updates the roadnetwork to scenario from the OpenDRIVE file
% "RRHighway.xodr" that was exported from RoadRunner and extracts
% laneInfo.

% Turnoff warnings to from open drive importer.
warning('off','driving:scenario:OpenDRIVEWarnings');

% Load OpenDrive file into driving scenario object
roadNetwork(scenario,'OpenDrive',"RRHighway.xodr");

% Define starting road segment id.
% For the shipping OpenDRIVE file "RRHighway.xodr" start segment ID is 6
startSegmentID = 1;

% Get number of road segments present in the scenario
numRoadSegments = size(scenario.RoadSegments, 2);

% Validate startSegmentID variable
if (startSegmentID > numRoadSegments || startSegmentID < 1)
    error("Invalid starting road segment id.");
end

% Call connectRoadSegments function to get concatenated road centers
% all segments present in the scenario and additionally get road segments
% sequence order
[roadCenters, ~] = connectRoadSegments(scenario, startSegmentID);

% Get number of lanes in the scenario
laneTypeSize = size(scenario.RoadSegments(startSegmentID).LaneType);
numLanes = laneTypeSize(2);

% Create lane width vector
laneWidth = zeros(1,numLanes);
for i = 1:numLanes
    laneWidth(i) = abs(scenario.RoadSegments(startSegmentID).CenterLaneBoundaryLocation{1}(i) - ...
                   scenario.RoadSegments(startSegmentID).CenterLaneBoundaryLocation{1}(i+1));
end

% Compute lane center information
laneInfo = estimateLaneCenters(roadCenters, laneWidth);

end

function [roadCenters, connectedOrder] = connectRoadSegments(scenario, startSegmentID)
%connectRoadSegments function outputs the concatenated road centers
% array and the connected road segments id sequence by taking scenario
% object and starting road segment id road as input

% Get number of road segments present in the scenario
numRoadSegments = size(scenario.RoadSegments, 2);

% Add start segment id to connected order array 
connectedOrder = startSegmentID;

% Add starting segment road centers to the road centers array
roadCenters = flip(scenario.RoadSegments(connectedOrder(end)).RoadCenters);

% Loop over the all road segments
for j = 1:numRoadSegments-1
    % Get road segment end point in each road segment
    % Using this end point we will find the next road segment based on
    % euclidean distance
    roadSegmentEndPoint = scenario.RoadSegments(connectedOrder(end)).RoadCenters(end, :);
    % Distance matrix contains the distance values:[segmentID, distance
    % between roadSegmentEndPoint and next road segment start point,
    % distance between roadSegmentEndPoint and next road segment end point]
    distMat = [];
    % Loop over the all road segements to find next nearest road segment
    for i=1:numRoadSegments
        % Check if the road segment id already exists in the connected
        % order array. If exists, don't add that road segment id to
        % distance matrix
        exists = 0;
        for k = 1:size(connectedOrder, 2)
            if i == connectedOrder(k)
                exists = 1;
                break;
            end
        end
        if exists == 1
            continue;
        end
        % Get segment start point from current iterated road segment
        segStartPoint = scenario.RoadSegments(i).RoadCenters(1, :);
        % Get segment end point from current iterated road segment
        segEndPoint = scenario.RoadSegments(i).RoadCenters(end, :);
        
        % Find euclidean distance between roadSegmentEndPoint and current road segment start point 
        startPointDist = norm(roadSegmentEndPoint - segStartPoint);
        % Find euclidean distance between roadSegmentEndPoint and current road segment end point
        endPointDist = norm(roadSegmentEndPoint - segEndPoint);
        
        % Update distance matrix which holds distances
        distMat = [distMat; [i, startPointDist, endPointDist]];
    end
    
    % Find minimum value and index from start point distances
    [minVal1, idx1] = min(distMat(:, 2));
    % Find minimum value and index from end point distances
    [minVal2, idx2] = min(distMat(:, 3));
     
    % Check whether or not to flip road centers and append road
    % centers to an array
    if (minVal1 > minVal2)
        connectedOrder = [connectedOrder, distMat(idx1, 1)];
        roadCenters = [roadCenters; flip(scenario.RoadSegments(connectedOrder(end)).RoadCenters)];
    else
        connectedOrder = [connectedOrder, distMat(idx2, 1)];
        roadCenters = [roadCenters; (scenario.RoadSegments(connectedOrder(end)).RoadCenters)];
    end
end

end