% Initialize the scenario 
scene = uavScenario(UpdateRate=100,ReferenceLocation=[0 0 0]);

%Create a ground for visualization
addMesh(scene,"polygon",{[-15 -15; 15 -15; 15 15; -15 15] [-0.5 0]},[0.3 0.3 0.3]);

% Add cylinder meshes to scan with lidar sensor
addMesh(scene,"cylinder",{[-5 5 2],[0 12]},[0 1 0]);
addMesh(scene,"cylinder",{[5 5 2],[0 12]},[0 1 0]);
addMesh(scene,"cylinder",{[5 -5 2],[0 12]},[0 1 0]);
% Platform/UAV initial position and orientation
initpos = [0 0 -5]; % NED Frame
initori = [0 0 0];

% Add two UAV Platform to the scenario and scale them for easier visualization
platform = uavPlatform("platformUAV",scene,ReferenceFrame="NED",...
    InitialPosition=initpos,InitialOrientation=eul2quat(initori));

updateMesh(platform,"quadrotor",{2},[0 0 0],eul2tform([0 0 pi]));

platform2 = uavPlatform("platformUAV2", scene, "ReferenceFrame", "NED", ...
    "InitialPosition", [0 7 -11], "InitialOrientation", eul2quat(initori));

updateMesh(platform2,"quadrotor", {2}, [0 0 0], eul2tform([0 0 pi]));

LidarModel = uavLidarPointCloudGenerator();
uavSensor("Lidar",platform,LidarModel,"MountingLocation",[0,0,1],"MountingAngles",[0 0 180]);
open_system("UAVScenarioModel.slx")


% Define the number of sensors and the ground station computer
num_sensors = 5;
ground_station = zeros(num_sensors, 2);

% Define the sensor types
sensor_types = {'Humidity', 'Temperature Variation', 'Soil Moisture Level', 'Nitrogen', 'Air Quality'};

% Define the drone's initial position and speed
drone_position = [0, 0];
drone_speed = 10; % meters per second

% Define the sensor locations
sensor_locations = rand(num_sensors, 2) * 1000; % Assuming a 1000x1000 meter field

% Implement the nearest neighbor algorithm to find the efficient path
unvisited_sensors = 1:num_sensors;
path = [];

while ~isempty(unvisited_sensors)
    % Find the nearest unvisited sensor
    [min_distance, nearest_sensor_idx] = min(sqrt(sum((sensor_locations(unvisited_sensors, :) - drone_position).^2, 2)));
    nearest_sensor_idx = unvisited_sensors(nearest_sensor_idx);
    
    % Add the nearest sensor to the path
    path = [path, nearest_sensor_idx];
    
    % Move the drone to the nearest sensor location
    travel_time = min_distance / drone_speed;
    drone_position = sensor_locations(nearest_sensor_idx, :);
    
    % Remove the visited sensor from the unvisited list
    unvisited_sensors(unvisited_sensors == nearest_sensor_idx) = [];
end

% Simulate the drone's data collection process
for i = 1:num_sensors
    % Move the drone to the next sensor location
    nearest_sensor_idx = path(i);
    travel_time = sqrt((sensor_locations(nearest_sensor_idx, 1) - drone_position(1))^2 + (sensor_locations(nearest_sensor_idx, 2) - drone_position(2))^2) / drone_speed;
    drone_position = sensor_locations(nearest_sensor_idx, :);
    
    % Collect data from the sensor
    sensor_type = sensor_types{nearest_sensor_idx};
    sensor_data = collect_sensor_data(sensor_type);
    
    % Store the data in the ground station computer
    ground_station(nearest_sensor_idx, 1) = sensor_data;
    ground_station(nearest_sensor_idx, 2) = toc; % Store the timestamp
   
end
 
% Display the collected data
for i = 1:num_sensors
    fprintf('Sensor Type: %s\n', sensor_types{i});
    fprintf('Sensor Data: %.2f\n',ground_station(i, 1));
    fprintf('Timestamp: %.2f seconds\n', ground_station(i, 2));
    fprintf('----------------------------\n');
end



% Helper function to collect sensor data (replace with actual sensor readings)
function sensor_data = collect_sensor_data(sensor_type)
    switch sensor_type
        case 'Humidity'
            sensor_data = rand() * 100; % Simulated humidity value
        case 'Temperature Variation'
            sensor_data = rand() * 50; % Simulated temperature variation value
        case 'Soil Moisture Level'
            sensor_data = rand() * 100; % Simulated soil moisture level value
        case 'Nitrogen'
            sensor_data = rand() * 1000; % Simulated nitrogen value
        case 'Air Quality'
            sensor_data = rand() * 10; % Simulated air quality value
    end
  
end