function [scenario, egoVehicle] = CreateOverpassScenario(scenario)

%% Description: 
% createDrivingScenario Returns the drivingScenario defined in the Designer
% Generated by MATLAB(R) 9.14 (R2023a) and Automated Driving Toolbox 3.7 (R2023a).
% Generated on: 16-Sep-2023 22:09:09

%% Code:
if scenario == "scenario1"
    % Construct a drivingScenario object.
    scenario = drivingScenario;
    
    % Add all road segments
    roadCenters = [-1.5 -0.2 0;
        71.7 0.1 0];
    marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
        laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
        laneMarking('Solid')];
    laneSpecification = lanespec(2, 'Width', 14.925, 'Marking', marking);
    road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');
    
    roadCenters = [47 37.1 0;
        53.7 1.3 0;
        47.1 -37.6 0];
    roadWidth = 15;
    road(scenario, roadCenters, roadWidth, 'Name', 'Road1');
    
    % Add the ego vehicle
    egoVehicle = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [5.6 -7.3 0], ...
        'Mesh', driving.scenario.carMesh, ...
        'Name', 'Car');
    
    % Add the non-ego actors
    car1 = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [38.1 6.3 0], ...
        'Mesh', driving.scenario.carMesh, ...
        'Name', 'Car1');
    waypoints = [38.1 6.3 0;
        33.4 6.2 0;
        30.1 5.1 0;
        25.7 6.2 0;
        21.5 6.3 0;
        17.4 6.5 0;
        14 6.2 0;
        9.4 6.2 0];
    speed = [30;30;30;30;30;30;30;30];
    trajectory(car1, waypoints, speed);
    
    bicycle = actor(scenario, ...
        'ClassID', 3, ...
        'Length', 1.7, ...
        'Width', 0.45, ...
        'Height', 1.7, ...
        'Position', [49.9 -23.7 0], ...
        'Mesh', driving.scenario.bicycleMesh, ...
        'Name', 'Bicycle');
    waypoints = [49.9 -23.7 0;
        50.7 -19.6 0;
        51.3 -13.5 0;
        52.4 -4.8 0;
        52 -0.8 0;
        52.4 4.5 0;
        52.1 10.3 0;
        51.6 14.7 0;
        51.2 18.3 0;
        50.5 22.1 0;
        49.9 24.6 0];
    speed = [15;15;15;15;15;15;15;15;15;15;15];
    trajectory(bicycle, waypoints, speed);


elseif scenario == "scenario2"
    % Construct a drivingScenario object.
    scenario = drivingScenario;
    
    % Add all road segments
    roadCenters = [-4.2 0 0;
        140.7 0.7 0];
    laneSpecification = lanespec(2, 'Width', 14.925);
    road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');
    
    % Add the ego vehicle
    egoVehicle = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [0.3 -5.5 0], ...
        'Mesh', driving.scenario.carMesh, ...
        'Name', 'Car');
    
    % Add the non-ego actors
    car1 = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [17.9 5.3 0], ...
        'Mesh', driving.scenario.carMesh, ...
        'Name', 'Car1');
    waypoints = [17.9 5.3 0;
        25.9 5.6 0;
        31 5.6 0;
        38.2 5.6 0;
        43.3 6.2 0;
        51.1 6 0;
        57.5 6.2 0];
    speed = [35;35;35;35;35;35;35];
    trajectory(car1, waypoints, speed);
    
    car2 = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [80.8 -7.3 0], ...
        'Mesh', driving.scenario.carMesh, ...
        'Name', 'Car2');
    waypoints = [80.8 -7.3 0;
        88.7 -6.9 0;
        94.5 -6.5 0;
        98.8 -6.9 0;
        105.2 -6.7 0;
        111.9 -6.7 0;
        117.7 -6.7 0];
    speed = [40;40;40;40;40;40;40];
    trajectory(car2, waypoints, speed);
end
end
