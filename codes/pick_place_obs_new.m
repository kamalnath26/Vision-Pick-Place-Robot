% Main Script for PincherX100 Pick-and-Place Operation
global Px100;

% Define the communication port for the robot
Px100.DEVICENAME = 'COM11'; % Replace 'COM11' with the correct port if needed

% Initialize the robot and its parameters
init_robot();

% Define threshold values and limits for robot operation
minZThreshold = Px100.MIN_Z_THRESH;  % Minimum allowed Z-coordinate (0.02 m)
maxMovementThreshold = Px100.MOVEMENT_THRESH; % Maximum movement distance (0.1 m)

% Specify pick, drop, and obstacle positions in the base frame
% pickPositionBase = trvec2tform([0.15, 0.1, 0.07]); % Example pick position
% dropPositionBase = trvec2tform([0.15, -0.1, 0.07]); % Example drop position
% obstaclePositionBase = trvec2tform([0.15, 0.1, 0.13]); % Example obstacle position (no obstacle)
[pickPositionBase, dropPositionBase, obstaclePositionBase] = get_pick_and_place_position();
pickPositionBase = trvec2tform([pickPositionBase(1,4),-pickPositionBase(2,4), 0.06]);
dropPositionBase = trvec2tform([dropPositionBase(1,4),-dropPositionBase(2,4), 0.06]);
obstaclePositionBase = trvec2tform([obstaclePositionBase(1,4),-obstaclePositionBase(2,4), 0.15]);
disp(pickPositionBase);
disp(dropPositionBase);
disp(obstaclePositionBase);

% Compute lift and approach positions relative to pick/drop positions
liftPositionBase = pickPositionBase * trvec2tform([0, 0, 0.13]); % 0.1 m above pick position
approachPositionBase = dropPositionBase * trvec2tform([0, 0, 0.13]); % 0.1 m above drop position

%% Phase 1: Picking the Payload
disp('Starting pick phase...');
while true
    % Capture the current joint positions as the home configuration
    currentJointPositions = get_joint_pos();
    home_configuration_position = currentJointPositions;

    liftJointConfig = computeInverseKinematics(Px100.robot_model, liftPositionBase);
    plannedTrajectory = planTrajectory(currentJointPositions, liftJointConfig);
    executeTrajectory(plannedTrajectory);

    % Compute the joint configuration to reach the pick position
    pickJointConfig = computeInverseKinematics(Px100.robot_model, pickPositionBase);
    plannedTrajectory = planTrajectory(liftJointConfig, pickJointConfig);

    % Execute the trajectory to reach the pick position
    executeTrajectory(plannedTrajectory);
    
     pause(2);
    % Close the gripper to grasp the object
    operateGripper(true);
    pause(2); % Small delay for gripper operation

    % Lift the payload to the lift position
    %liftJointConfig = computeInverseKinematics(Px100.robot_model, liftPositionBase);
    plannedTrajectory = planTrajectory(pickJointConfig, liftJointConfig);
    executeTrajectory(plannedTrajectory);

    break; % End pick phase
end

%% Phase 2: Transporting the Payload
% disp('Starting transport phase...');
% while true
%     % Compute trajectory to approach position above the drop point
%     approachJointConfig = computeInverseKinematics(Px100.robot_model, approachPositionBase);
%     plannedTrajectory = planTrajectory(liftJointConfig, approachJointConfig);
%     executeTrajectory(plannedTrajectory);
% 
%     % Compute trajectory to the final drop position
%     dropJointConfig = computeInverseKinematics(Px100.robot_model, dropPositionBase);
%     plannedTrajectory = planTrajectory(approachJointConfig, dropJointConfig);
%     executeTrajectory(plannedTrajectory);
% 
%     % Open the gripper to release the payload
%     pause(2);
%     operateGripper(false);
%     pause(2); % Small delay for gripper operation
% 
% 
%     up_JointConfig_p = approachJointConfig;
%     plannedTrajectory = planTrajectory(dropJointConfig, up_JointConfig_p);
%     executeTrajectory(plannedTrajectory);
% 
%     break; % End transport phase
% end

%% Phase 2: Transporting the Payload
disp('Starting transport phase...');
while true
    % Check if an obstacle exists (not at origin)
    if ~isequal(obstaclePositionBase, trvec2tform([0, 0, 0]))

        % Extract the x and y coordinates from the obstacle position
        obstaclePos = tform2trvec(obstaclePositionBase);
        obstacleX = obstaclePos(1);
        obstacleY = obstaclePos(2);

        % Determine the buffer based on the obstacle position
        if abs(obstacleX - 0.2) < 0.02 && abs(obstacleY - 0.2) < 0.02
            % If obstacle is close to [0.2, 0.2], subtract buffer
            buffer_x = -0.1; % Subtract 5 cm from x
            buffer_y = -0.1; % Subtract 5 cm from y
        elseif abs(obstacleX - 0.1) < 0.02 && abs(obstacleY - 0.1) < 0.02
            % If obstacle is close to [0.1, 0.1], add buffer
            buffer_x = 0.1; % Add 5 cm to x
            buffer_y = 0.1; % Add 5 cm to y
        elseif abs(obstacleX) < 0.02 && abs(obstacleY) < 0.02
            % If obstacle is close to [0, 0], add buffer
            buffer_x = 0.1; % Add 5 cm to x
            buffer_y = 0.1; % Add 5 cm to y
        else
            % Default buffer values if none of the above conditions match
            buffer_x = 0.1; % Default buffer for x
            buffer_y = 0.1; % Default buffer for y
        end

        % % Add a 5 cm buffer to the obstacle position
        % buffer_x = 0.1; % 5 cm
        % buffer_y = 0.1; % 5 cm
        bufferedObstaclePositionBase = obstaclePositionBase * trvec2tform([buffer_x, buffer_y, 0.16]); % Example: buffer in x-direction
        
        % Generate inverse kinematics for buffered obstacle position
        obstacleAvoidanceJointConfig = computeInverseKinematics(Px100.robot_model, bufferedObstaclePositionBase);
        plannedTrajectory = planTrajectory(liftJointConfig, obstacleAvoidanceJointConfig);
        executeTrajectory(plannedTrajectory);

        % Move from buffered obstacle position to drop approach position
        approachJointConfig = computeInverseKinematics(Px100.robot_model, approachPositionBase);
        plannedTrajectory = planTrajectory(obstacleAvoidanceJointConfig, approachJointConfig);
        executeTrajectory(plannedTrajectory);
    else
        % No obstacle, move directly to drop approach position
        approachJointConfig = computeInverseKinematics(Px100.robot_model, approachPositionBase);
        plannedTrajectory = planTrajectory(liftJointConfig, approachJointConfig);
        executeTrajectory(plannedTrajectory);
    end

    % Compute trajectory to the final drop position
    dropJointConfig = computeInverseKinematics(Px100.robot_model, dropPositionBase);
    plannedTrajectory = planTrajectory(approachJointConfig, dropJointConfig);
    executeTrajectory(plannedTrajectory);

    % Open the gripper to release the payload
    pause(2);
    operateGripper(false);
    pause(2); % Small delay for gripper operation

    % Return to the approach position
    up_JointConfig_d = approachJointConfig;
    plannedTrajectory = planTrajectory(dropJointConfig, approachJointConfig);
    executeTrajectory(plannedTrajectory);

    break; % End transport phase
end


%% Phase 3: Returning to Home Position
disp('Returning to the home position...');
while true
    % Plan and execute trajectory to return to the home configuration
    plannedTrajectory = planTrajectory(up_JointConfig_d, home_configuration_position);
    executeTrajectory(plannedTrajectory);

    break; % End return phase
end

disp('Pick-and-place task completed successfully.');

%% Supporting Functions

% Function to plan a smooth trajectory between two joint configurations
function waypoints = planTrajectory(startConfig, endConfig)
    numWaypoints = 100; % Number of points in the trajectory
    waypoints = zeros(numWaypoints, length(startConfig));
    for idx = 1:length(startConfig)
        waypoints(:, idx) = linspace(startConfig(idx), endConfig(idx), numWaypoints);
    end
end

% Function to compute inverse kinematics for a target pose
function jointConfig = computeInverseKinematics(robotModel, targetPose)
    ikSolver = inverseKinematics('RigidBodyTree', robotModel);
    weights = [1 0 0 1 1 1]; % IK weight parameters
    initialGuess = homeConfiguration(robotModel);
    [solution, ~] = ikSolver('px100/ee_gripper_link', targetPose, weights, initialGuess);
    jointConfig = [solution.JointPosition]; 
    jointConfig = jointConfig(1:4);% Use first 4 joints only
end

% Function to execute a planned trajectory
function executeTrajectory(trajectoryWaypoints)
    global Px100;
    numWaypoints = size(trajectoryWaypoints, 1);

    for idx = 2:numWaypoints
        successful = set_joint_pos(trajectoryWaypoints(idx, :));
        while ~successful
            % Wait until the joint position is successfully updated
            pause(0.01);
            successful = set_joint_pos(trajectoryWaypoints(idx, :));
        end
        pause(0.02); % Small delay for smooth execution
    end
end

% Function to control the gripper (open/close)
function operateGripper(data)
    % Pass `true` to close the gripper and `false` to open it
    if data
        closeGripper(true);
    else
        closeGripper(false);
    end
end

