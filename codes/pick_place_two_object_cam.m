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
% pickPositionBase_2 = trvec2tform([0.15, 0.1, 0.07]); % Example pick position
% dropPositionBase_2 = trvec2tform([0.15, -0.1, 0.13]); % Example drop position
% obstaclePositionBase = trvec2tform([0, 0, 0]); % Example obstacle position (no obstacle)

[pickPositionBase, dropPositionBase, obstaclePositionBase] = get_pick_and_place_position();
pickPositionBase = trvec2tform([pickPositionBase(1,4),-pickPositionBase(2,4), 0.06]);
dropPositionBase = trvec2tform([dropPositionBase(1,4),-dropPositionBase(2,4), 0.06]);
dropPositionBase_2 = trvec2tform([dropPositionBase(1,4),-dropPositionBase(2,4), 0.06]);
obstaclePositionBase = trvec2tform([obstaclePositionBase(1,4),-obstaclePositionBase(2,4), 0.04]);
disp(pickPositionBase);
disp(dropPositionBase);
disp(obstaclePositionBase);

% Compute lift and approach positions relative to pick/drop positions
liftPositionBase = pickPositionBase * trvec2tform([0, 0, 0.16]); % 0.1 m above pick position
approachPositionBase = dropPositionBase * trvec2tform([0, 0, 0.16]); % 0.1 m above drop position


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
disp('Starting transport phase...');
while true
    % Compute trajectory to approach position above the drop point
    approachJointConfig = computeInverseKinematics(Px100.robot_model, approachPositionBase);
    plannedTrajectory = planTrajectory(liftJointConfig, approachJointConfig);
    executeTrajectory(plannedTrajectory);

    % Compute trajectory to the final drop position
    dropJointConfig = computeInverseKinematics(Px100.robot_model, dropPositionBase);
    plannedTrajectory = planTrajectory(approachJointConfig, dropJointConfig);
    executeTrajectory(plannedTrajectory);

    % Open the gripper to release the payload
    pause(2);
    operateGripper(false);
    pause(2); % Small delay for gripper operation


    up_JointConfig = approachJointConfig;
    plannedTrajectory = planTrajectory(dropJointConfig, up_JointConfig);
    executeTrajectory(plannedTrajectory);

    break; % End transport phase
end

%% Phase 3: Returning to Home Position
disp('Returning to the home position...');
while true
    % Plan and execute trajectory to return to the home configuration
    plannedTrajectory = planTrajectory(up_JointConfig, home_configuration_position);
    executeTrajectory(plannedTrajectory);

    break; % End return phase
end

disp('Pick-and-place task completed successfully.');


pause(2);

disp("starting object position detection for object 2 ")

% [pickPositionBase, dropPositionBase, obstaclePositionBase] = get_pick_and_place_position();
% pickPositionBase = trvec2tform([obstaclePositionBase(1,4),-obstaclePositionBase(2,4), 0.06]);
pickPositionBase = obstaclePositionBase;
% dropPositionBase = trvec2tform([dropPositionBase(1,4),-dropPositionBase(2,4), 0.13]);
dropPositionBase = dropPositionBase_2;
%obstaclePositionBase = trvec2tform([obstaclePositionBase(1,4),-obstaclePositionBase(2,4), 0.15]);
disp(pickPositionBase);
disp(dropPositionBase);
% disp(obstaclePositionBase);

liftPositionBase_2 = pickPositionBase * trvec2tform([0, 0, 0.16]); % 0.1 m above pick position
approachPositionBase_2 = dropPositionBase_2 * trvec2tform([0, 0, 0.16]); % 0.1 m above drop position

%%%%%%%%%%%%%%%%%%%%%%%%%% 2nd object pick and place %%%%%%%%%%%%%%%%%%%%
%% Phase 1: Picking the Payload
disp('Starting pick phase... for object 2');
while true
    % Capture the current joint positions as the home configuration
    currentJointPositions = get_joint_pos();
    home_configuration_position = currentJointPositions;

    liftJointConfig = computeInverseKinematics(Px100.robot_model, liftPositionBase_2);
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
disp('Starting transport phase...');
while true
    % Compute trajectory to approach position above the drop point
    approachJointConfig = computeInverseKinematics(Px100.robot_model, approachPositionBase_2);
    plannedTrajectory = planTrajectory(liftJointConfig, approachJointConfig);
    executeTrajectory(plannedTrajectory);

    % Compute trajectory to the final drop position
    dropJointConfig = computeInverseKinematics(Px100.robot_model, dropPositionBase_2);
    plannedTrajectory = planTrajectory(approachJointConfig, dropJointConfig);
    executeTrajectory(plannedTrajectory);

    % Open the gripper to release the payload
    pause(2);
    operateGripper(false);
    pause(2); % Small delay for gripper operation


    up_JointConfig = approachJointConfig;
    plannedTrajectory = planTrajectory(dropJointConfig, up_JointConfig);
    executeTrajectory(plannedTrajectory);

    break; % End transport phase
end

%% Phase 3: Returning to Home Position
disp('Returning to the home position...');
while true
    % Plan and execute trajectory to return to the home configuration
    plannedTrajectory = planTrajectory(up_JointConfig, home_configuration_position);
    executeTrajectory(plannedTrajectory);

    break; % End return phase
end

disp('Pick-and-place task completed successfully for object 2.');



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

