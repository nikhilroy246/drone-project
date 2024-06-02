clearvars;
close all;

% Define field dimensions, row spacing, and altitude
fieldLength = 100;  % meters
fieldWidth = 100;    % meters
rowSpacing = 10;    % meters
altitude = 15;      % meters

% Generate waypoints
waypoints = generateFieldWaypoints3D(fieldLength, fieldWidth, rowSpacing, altitude);

% Display waypoints
disp(waypoints);
s_waypoints_e = [0 0 0 ;waypoints; fieldLength fieldWidth 0 ];
[fr,fclose]=size(s_waypoints_e);

% Simulate
plantDataDictionary = Simulink.data.dictionary.open('pathFollowingData.sldd');
plantDataSet = getSection(plantDataDictionary,'Design Data');
navigationModel = 'pathFollowing';
open_system(navigationModel);
assignin(plantDataSet,'useHighFidelity',0);
sim(navigationModel);
% Visualize
p=visualizeSimStates(simStates);
%%
waypointData = collectDataAtWaypoints(waypoints);
function waypoints = generateFieldWaypoints3D(fieldLength, fieldWidth, rowSpacing, altitude)
waypoints = [];
x = 0:rowSpacing:fieldLength;
numRows = length(x);

for i = 1:numRows
    if mod(i, 2) == 1

        waypoints = [waypoints; x(i), 0, altitude];
        waypoints = [waypoints; x(i), fieldWidth, altitude];
    else

        waypoints = [waypoints; x(i), fieldWidth, altitude];
        waypoints = [waypoints; x(i), 0, altitude];
    end
end
end

function waypointData = collectDataAtWaypoints(waypoints)
% Generate waypoints
%waypoints = generateFieldWaypoints3D(fieldLength, fieldWidth, rowSpacing, altitude);

% Initialize data storage
waypointData = struct('waypoint', [], 'sensorData', []);

% Simulate drone movement and data collection
for i = 1:size(waypoints, 1)
    % Simulate moving to the waypoint
    currentWaypoint = waypoints(i, :);

    % Collect sensor data (simulate with random data for this example)
    sensorData = getSensorData(currentWaypoint);

    % Store data
    waypointData(i).waypoint = currentWaypoint;
    waypointData(i).sensorData = sensorData;

    % Display current waypoint and collected data
    fprintf('Waypoint: [%.2f, %.2f, %.2f]\n', currentWaypoint);
    fprintf('Sensor Data: %.2f\n', sensorData);
end
% Extract waypoints and sensor data for plotting
waypointsArray = reshape([waypointData.waypoint], 3, []).';
sensorDataArray = [waypointData.sensorData];
% Plot the waypoints and sensor data
%figure;
plot3(waypointsArray(:,1), waypointsArray(:,2), waypointsArray(:,3), 'bo-');
hold on;
scatter3(waypointsArray(:,1), waypointsArray(:,2), sensorDataArray, 'filled');

% Annotate the plot with sensor data
for i = 1:length(sensorDataArray)
    text(waypointsArray(i,1), waypointsArray(i,2), waypointsArray(i,3), ...
        sprintf('%.2f', sensorDataArray(i)), ...
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
end
title('3D Agricultural Field Waypoints and Sensor Data');

xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Altitude (meters)');
grid on;
axis equal;
end

function sensorData = getSensorData(~)
% Simulate sensor data collection
% Replace this with actual sensor data collection logic
sensorData = rand(); % Random data for simulation
end



