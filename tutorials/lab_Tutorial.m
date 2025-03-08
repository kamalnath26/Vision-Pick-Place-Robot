%% Importing all the necessary libraries 
clear, clc, close all

% Load the Dynamixel library
lib_name = '';
if strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', ...
        'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h', ...
        'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

%% Setting constants

% Define motor and communication settings
DXL_ID = [1, 2, 3, 4, 5];
BAUDRATE = 1000000;         
DEVICENAME = 'COM8';

% Storing byte addresses
ADDR_OPERATING_MODE = 11;
ADDR_TORQUE_ENABLE = 64;
ADDR_GOAL_VELOCITY = 104;  
ADDR_GOAL_POS = 116;
ADDR_PRESENT_VELOCITY = 128;
ADDR_PRESENT_POSITION = 132;

% Values of the bytes
LEN_GOAL_VELOCITY = 4;
LEN_PRESENT_VELOCITY = 4;
LEN_PRESENT_POSITION = 4;   
VELOCITY_MODE = 1;          
POSITION_MODE = 3;
PROTOCOL_VERSION = 2.0;
TORQUE_ENABLE = 1;
TORQUE_DISABLE = 0;

COMM_SUCCESS = 0;            % Communication Success result value
COMM_TX_FAIL = -1001;        % Communication Tx Failed

%% Initializing Comms with robot

% Initialize PortHandler and PacketHandler
port_num = portHandler(DEVICENAME);
packetHandler();

% Opening port
if openPort(port_num)
    fprintf('Succeeded to open the port!\n');
else
    fprintf('Failed to open the port!\n');
    unloadlibrary(lib_name);
    return;
end

% Setting baud-rate
if setBaudRate(port_num, BAUDRATE)
    fprintf('Succeeded to change the baudrate!\n');
else
    fprintf('Failed to change the baudrate!\n');
    unloadlibrary(lib_name);
    return;
end

%% Initializing params and getting robot to home configuration using position control mode

% Initialize groupBulkWrite Struct
groupwrite_num = groupBulkWrite(port_num, PROTOCOL_VERSION);

% Initialize Groupbulkread Structs
groupread_num = groupBulkRead(port_num, PROTOCOL_VERSION);

% Joint positions for Home config
goal_positions = [2050, 1250, 2500, 2350, 2500];

% Moving robot to Home configuration
for i = 1:length(DXL_ID)

    % Setting the operation mode to position control and enabling torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR_OPERATING_MODE, POSITION_MODE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    
    % Sending the joint position commands
    param_goal_position = typecast(int32(goal_positions(i)), 'uint32');
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR_GOAL_POS, param_goal_position);
end

pause(0.5) % Wait for the robot to reach Home config

for i = 1:length(DXL_ID)
    
    % Disabling torque(to change operation mode, torque needs to be disabled)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    
    % Setting the operating mode to velocity control and re-enabling torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR_OPERATING_MODE, VELOCITY_MODE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    
    % Creating the Bulkread template
    groupBulkReadAddParam(groupread_num, DXL_ID(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
end

%% Setting joint-position limits

JOINT_LIMITS = [1000, 3050;
                 950, 3060;
                 950, 3060;
                 830, 3400;
                1850, 2700];

%% Setting joint velocity using velocity control mode

% Example velocities for testing each motor
goal_velocities = [20, 20, -20, -20, -20];

timeout = 5;  % Duration in seconds to hold each velocity command
update_freq = 10; % in Hz
time_period = 1/update_freq;

tic;
tStart = toc;
tUpdate = toc;

% Initializing flags
FIRST_RUN = true;
STOP_FLAG = false;

while(true)
    t2 = toc;
    
    if (t2 - tUpdate > time_period || FIRST_RUN)
        fprintf("\n--------\nUpdating velocity!\n--------\n\n");
        for i = 1:length(DXL_ID)
            param_goal_velocity = typecast(int32(goal_velocities(i)), 'uint32');
            groupBulkWriteAddParam(groupwrite_num, DXL_ID(i), ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, param_goal_velocity, LEN_GOAL_VELOCITY);
        end

        % Writing the bulk packet
        groupBulkWriteTxPacket(groupwrite_num);
        groupBulkWriteClearParam(groupwrite_num);

        % Resetting tUpdate
        tUpdate = toc;
        FIRST_RUN = false;
    end

    % Reading the packet
    groupBulkReadTxRxPacket(groupread_num);

    % Printing the joint positions
    for i=1:length(DXL_ID)
        % Reading the joint positions
        val_read = groupBulkReadGetData(groupread_num, DXL_ID(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        fprintf('[ID:%03d] JointPos:%d\n', DXL_ID(i), typecast(uint32(val_read), 'int32'));
        
        % Checking for joint limit violations
        if(val_read < JOINT_LIMITS(i, 1) || val_read > JOINT_LIMITS(i, 2))
            STOP_FLAG = true;
            break
        end
    end
    fprintf("------x------\n");

    t = toc;
    
    % Exit condition
    if(t - tStart > timeout || STOP_FLAG)
        if STOP_FLAG
            fprintf("\n\n==========x==========\n\nWARNING!\nJoint limit exceeded!\n")
        else
            fprintf("\n\n==========x==========\n\nTimeout!\n");
        end
        break
    end
end

for i=1:length(DXL_ID)
    % Stop the motor after the duration by setting velocity to 0
    param_goal_velocity = typecast(int32(0), 'uint32');
    groupBulkWriteAddParam(groupwrite_num, DXL_ID(i), ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, param_goal_velocity, LEN_GOAL_VELOCITY);
end

% Writing the bulk packet
groupBulkWriteTxPacket(groupwrite_num);
groupBulkWriteClearParam(groupwrite_num);

%% Disable Torque and close port

prompt = "Enter any key to disable: ";
input(prompt, "s");

% Disabling the torque
for i = 1:length(DXL_ID)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
end

%% Closing the port and unloading the libraries

closePort(port_num);
unloadlibrary(lib_name);
 