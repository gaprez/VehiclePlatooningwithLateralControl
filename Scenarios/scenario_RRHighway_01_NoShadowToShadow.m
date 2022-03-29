function [scenario,assessment] = scenario_RRHighway_01_NoShadowToShadow(T,Ts,speed)
% scenario_RRHighway_01_NoShadowToShadow creates a driving scenario by
% importing the OpenDRIVE file from the RoadRunner scene. This scenario
% contains five other vehicles in the scene. In this scenario the vehicles
% travel from a road segment where the trees do not cast shadows on the
% road to the segment where the trees cast shadows on lane markings of the
% road.

%   Copyright 2020 The MathWorks, Inc.

% Get the scenario object from OpenDRIVE file, corresponding laneInfo and
% assessment
[scenario, assessment, laneInfo] = helperGetLaneFollowingScenario("RR Highway road",T,Ts);

% Ego and Target Vehicles representation in this test case.
%       Actors(1) - EgoCar
%       Actors(2) - LeadCar
%       Actors(3) - SlowCar1
%       Actors(4) - SlowCar2
%       Actors(5) - FastCar1
%       Actors(6) - FastCar2

%% EgoCar: Set position, speed using trajectory
% Add Trajectory for EgoCar. This vehicle travels in lane number 5 and
% starts at 1050 meters from the initial waypoint. Set velocity of the
% vehicle to 10 m/s

egoCar = scenario.Actors(1);

% Get waypoints from laneCenters.  
% Place the EgoCar in Lane5.
waypoints = laneInfo(5).LaneCenters;

% Find the start waypoint for EgoCar at a distance of 0 meters from
% initial waypoint.
startDistanceFromFirstWaypoint = 0;
[~, ~, waypointStartIndex]...
    = helperGetPositionFromWaypoint(waypoints,startDistanceFromFirstWaypoint);

% Find the end waypoint for EgoCar at a distance of speed*T meters from
% initial waypoint.
endDistanceFromFirstWaypoint = speed*T;
[~, ~, waypointEndIndex]...
    = helperGetPositionFromWaypoint(waypoints,endDistanceFromFirstWaypoint);
waypointsEgoCar = waypoints(waypointStartIndex:waypointEndIndex,:);

% Set trajectory for EgoCar
trajectory(egoCar, waypointsEgoCar, speed);

%% Set Simulation stop time
scenario.StopTime = T;

% Explore the scenario using Driving Scenario Designer
%drivingScenarioDesigner(scenario)
