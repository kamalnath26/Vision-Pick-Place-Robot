% Global Robot Structure
global Px100;
global joint_positions pick_pose_base drop_pose_base;

% Define pick and drop poses
pick_pose_base = trvec2tform([0.27, 0.10, 0.055]);  % Default pick pose ( You can change it)
drop_pose_base = trvec2tform([0.27, -0.10, 0.055]);  % Default drop pose ( You can change it)

% Initialize simulation environment
init_robot_sim(pick_pose_base, drop_pose_base);

%% Main Loop
try
    % Call main task
    main_task();
catch ME
    disp('Error encountered:');
    disp(ME.message);
end

function main_task()
    global joint_positions Px100 pick_pose_base drop_pose_base;
    
    %%
    %% Reaching the Payload
    while true
        % Get current joint positions (first 4 only)
        joint_positions = get_current_joint_positions();
        disp(["got joint positions:", joint_positions]);
        disp(['Type of joint_positions: ', class(joint_positions)]);
        
        %Write Inverse Kinematics for PincherX100. The input should take the pick_pose and the output of it should be the pick_joint_config( joint configuration wrt pick_pose)
        %%
        pick_joint_config = inverse_kinematics(pick_pose_base * trvec2tform([0.25, 0.08, 0.55]));
        disp(["found IK:", pick_joint_config]);
        disp(["Type of IK:", class(pick_joint_config)]);

        % Write the code for trajectory planning for reaching the payload (pick_pose). The input should be the current pose, pick_pose. The output should be "joint_space_waypoints" n x 4 matrix.
        %%
        joint_space_waypoints = plan_trajectory(joint_positions, pick_joint_config);
        disp("joint_space_waypoints:");
        disp(joint_space_waypoints);
        disp(["Type of waypoints:", class(joint_space_waypoints)]);

        % Execute trajectory (first 4 joints only)
        execute_trajectory(joint_space_waypoints);

        % Attach cylinder at pick pose
        attachCylinder();
        pause(0.5);

        lift_pose = pick_pose_base * trvec2tform([0.8, 0, 0.05]);  % 5cm upward movement
        lift_joint_config = inverse_kinematics(lift_pose);
        lift_waypoints = plan_trajectory(pick_joint_config, lift_joint_config);
        
        execute_trajectory(lift_waypoints);
        
        break;  % Exit loop after first run for simplicity
    end

    %% Transporting Payload
    while true
        %Write Inverse Kinematics for PincherX100 . The input should take the drop_pose and the output of it should be the drop_joint_config( joint configuration wrt drop_pose)
        %%
        drop_joint_config = inverse_kinematics(drop_pose_base * trvec2tform([0.25, -0.08, 0.55]));

        % Write the code for trajectory planning from pick_pose to drop_pose. The input should be the pick_pose, drop_pose. The output should be "joint_space_waypoints" n x 4 matrix.
        %%
        joint_space_waypoints = plan_trajectory(lift_joint_config, drop_joint_config);

        % Execute trajectory
        execute_trajectory(joint_space_waypoints);

        % Detach cylinder at drop pose
        detachCylinder();

        pause(0.5);
        break;
    end

    %% Returning to Home Configuration
    while true
        % Write the code for trajectory planning from drop_pose to home_pose. The input should be the drop_pose, home.Config . The output should be "joint_space_waypoints" n x 4 matrix.
        %%
        home_joint_config = Px100.homeConfig(1:4);
        joint_space_waypoints = plan_trajectory(drop_joint_config, home_joint_config);

        % Execute trajectory
        execute_trajectory(joint_space_waypoints);
        break;
    end
end
%%
function joint_config = inverse_kinematics(target_pose)
    global Px100;
    
    % End-effector offset
    ee_offset = 0.027575 ; %  0.030575

    % Extract position from target pose
    position = target_pose(1:3, 4)- target_pose(1:3, 1:3) * [ee_offset; 0; 0];
    
    % Calculate joint angles
    theta1 = atan2(position(2), position(1));
    
    r = sqrt(position(1)^2 + position(2)^2);
    z = position(3);
    
    % Simplified calculations for theta2 and theta3
    l2 = 0.1;  % Length of link 2 (adjust as needed)
    l3 = 0.1;  % Length of link 3 (adjust as needed)
    
    cos_theta3 = (r^2 + z^2 - l2^2 - l3^2) / (2 * l2 * l3);
     cos_theta3 = max(-1, min(1, cos_theta3));  % Clamp to [-1, 1]
    theta3 = atan2(sqrt(1 - cos_theta3^2), cos_theta3);
    
    alpha = atan2(z, r);
    beta = atan2(l3 * sin(theta3), l2 + l3 * cos(theta3));
    theta2 = alpha - beta;
    
    % Calculate theta4 to keep z_e aligned with z_0
    theta4 = -theta2 - theta3;
    
    joint_config = [theta1, theta2, theta3, theta4];
end

function waypoints = plan_trajectory(start_config, end_config)
    num_waypoints = 30;  % Increased from 10 to 100
    waypoints = zeros(num_waypoints, 4);
    % Via point above the pick/drop location
    via_config = start_config;
    via_config(3) = via_config(3) - pi/4;  % Adjust elbow joint
    
    % Generate waypoints for start to via, and via to end
    for i = 1:num_waypoints/2
        t = (i - 1) / (num_waypoints/2 - 1);
        waypoints(i, :) = (1 - t) * start_config + t * via_config;
        waypoints(i + num_waypoints/2, :) = (1 - t) * via_config + t * end_config;
    end

    for i = 1:num_waypoints
        t = (i - 1) / (num_waypoints - 1);
        waypoints(i, :) = (1 - t) * start_config + t * end_config;
    end
end
%%
% Setting up simulation environment
function init_robot_sim(pick_pose, drop_pose)
    global Px100;
    Px100.robot = importrobot('px100.urdf');% path to the urdf file
    Px100.robot.DataFormat = 'row';
    Px100.homeConfig = homeConfiguration(Px100.robot);
    Px100.currentConfig = Px100.homeConfig;

    % Display the robot in its home configuration
    figure;
    show(Px100.robot, Px100.homeConfig, 'PreservePlot', false, 'Collisions', 'on');
    hold on;

    % Initialize environment with platform and cylinder
    Px100.env = {};

    % Add platform
    platform = collisionBox(0.6, 1, 0.02);
    platform.Pose = trvec2tform([0, 0, -0.01]);
    Px100.env{1} = platform;

    % Add cylinder at pick pose
    cylinderRadius = 0.03;
    cylinderHeight = 0.05;
    cylinder = collisionCylinder(cylinderRadius, cylinderHeight);
    cylinder.Pose = pick_pose * trvec2tform([0, 0, -cylinderHeight / 2]); % Centered at pick position
    Px100.env{2} = cylinder;

    % Add pick and drop markers
    add_markers(pick_pose, drop_pose);

    % Display environment objects
    for i = 1:length(Px100.env)
        show(Px100.env{i});
    end
end
% Placing markers at Pick and drop position
function add_markers(pickPose, dropPose)
    global Px100;

    % Add pick marker
    pickMarker = collisionSphere(0.01);
    pickMarker.Pose = pickPose;
    Px100.env{3} = pickMarker;

    % Add drop marker
    dropMarker = collisionSphere(0.01);
    dropMarker.Pose = dropPose;
    Px100.env{4} = dropMarker;

    % Display markers
    show(pickMarker);
    show(dropMarker);
end

%Imposing joint limit constraints and minimum Z threshold
function execute_trajectory(waypoints)
    global Px100;

    % Define joint limits
    joint_limits = [
        -1.6, 1.53;  % Joint 1
        -1.68, 1.55; % Joint 2
        -1.68, 1.55; % Joint 3
        -1.86, 2.07  % Joint 4
    ];

    % Minimum allowed z position
    min_z_thresh = 0.01; % 0.02 m

    % Maximum allowed distance between consecutive waypoints
    movement_thresh = 0.15;  % 0.1 m

    for i = 1:size(waypoints, 1)
        % Apply joint limits for each joint
        for j = 1:4
            waypoints(i, j) = max(joint_limits(j, 1), min(joint_limits(j, 2), waypoints(i, j)));
        end

        % Calculate the forward kinematics for the current configuration
        Px100.currentConfig(1:4) = waypoints(i, :);
        eePose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');

        % Check the z position constraint
        if eePose(3, 4) < min_z_thresh
            error('Z position constraint violated at waypoint %d. Execution stopped.', i);
        end

        % Check distance between waypoints
        if i > 1
            dist = norm(waypoints(i, :) - waypoints(i - 1, :));
            if dist > movement_thresh
                error('Distance between consecutive waypoints exceeds threshold at waypoint %d. Execution stopped.', i);
            end
        end

        % Show the robot's configuration
        show(Px100.robot, Px100.currentConfig, 'PreservePlot', false, 'Collisions', 'on');
        drawnow;
    end
end

function attachCylinder()
    global Px100;
    if isempty(Px100.env{2})
        disp('Cylinder is already attached or does not exist.');
        return;
    end

    cylinder = Px100.env{2};
    cylinderBody = rigidBody('cylinderBody');
    cylinderJoint = rigidBodyJoint('cylinderJoint', 'fixed');

    % Get end-effector pose at the current config
    eePose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');
    setFixedTransform(cylinderJoint, eePose \ cylinder.Pose);
    addCollision(cylinderBody, cylinder, inv(cylinder.Pose));
    cylinderBody.Joint = cylinderJoint;
    addBody(Px100.robot, cylinderBody, 'px100/ee_gripper_link');

    % Remove the cylinder from the environment to indicate it is picked up
    Px100.env{2} = [];
end

function detachCylinder()
    global Px100;

    % Remove cylinder from robot
    removeBody(Px100.robot, 'cylinderBody');

    % Get the end-effector pose at the current configuration
    dropPose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');

    % Define cylinder dimensions
    cylinderRadius = 0.03;
    cylinderHeight = 0.05;

    % Create a new cylinder collision object
    cylinder = collisionCylinder(cylinderRadius, cylinderHeight);

    % Adjust cylinder pose relative to the drop pose
    cylinder.Pose = dropPose * trvec2tform([0, 0, -cylinderHeight / 2]);

    % Re-add the cylinder to the environment at the new pose
    Px100.env{2} = cylinder;

    % Display the updated environment for verification
    show(Px100.env{2});
end

function joint_positions = get_current_joint_positions()
    global Px100;
    % Read only the first 4 joint positions
    joint_positions = Px100.currentConfig(1:4);
end

