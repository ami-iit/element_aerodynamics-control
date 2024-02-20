%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON ROBOT CONFIGURATION PARAMETERS                      %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% GENERAL ROBOT MODEL INFORMATION

if Config.experimentCase == 1
    Config.N_DOF     = 17; % flying
elseif Config.experimentCase == 0
    Config.N_DOF     = 22;%23; % hovering
end

Config.N_DOF_MATRIX  = eye(Config.N_DOF);
Config.ON_GAZEBO     = false;
Config.GRAVITY_ACC   = 9.81;

% 4 element list identifying jets'axes: The value can be either 1,2,3 and
% it identifies the axes x,y,z of the associated end effector frame. The
% sign identifies the direction.
Config.jets.axes     =  zeros(4,1);
Config.jets.axes(1)  = -3;
Config.jets.axes(2)  = -3;
Config.jets.axes(3)  = -3;
Config.jets.axes(4)  = -3;

% Robot configuration for WBToolbox
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icub';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% Controlboards and joints list. Each joint is associated to the corresponding controlboard
WBTConfigRobot.ControlBoardsNames     = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints       = [];
Config.numOfJointsForEachControlboard = [];

ControlBoards = struct();

if Config.experimentCase == 1 % flying
    ControlBoards.(WBTConfigRobot.ControlBoardsNames{1}) = {'torso_yaw'};
    ControlBoards.(WBTConfigRobot.ControlBoardsNames{4}) = {'l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
    ControlBoards.(WBTConfigRobot.ControlBoardsNames{5}) = {'r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
elseif Config.experimentCase == 0 % hovering
    ControlBoards.(WBTConfigRobot.ControlBoardsNames{1}) = {'torso_pitch','torso_yaw'};%{'torso_pitch','torso_roll','torso_yaw'};
    ControlBoards.(WBTConfigRobot.ControlBoardsNames{4}) = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
    ControlBoards.(WBTConfigRobot.ControlBoardsNames{5}) = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
end

ControlBoards.(WBTConfigRobot.ControlBoardsNames{2}) = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{3}) = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow'};

for n = 1:length(WBTConfigRobot.ControlBoardsNames)

    WBTConfigRobot.ControlledJoints       = [WBTConfigRobot.ControlledJoints, ControlBoards.(WBTConfigRobot.ControlBoardsNames{n})];
    Config.numOfJointsForEachControlboard = [Config.numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobot.ControlBoardsNames{n}))];
end

%% JOINT LIMITS

% Joints limits list. Limits are scaled by a safety range (smaller than the
% normal joints limit range)
scaleTorsoJointsLimits = 1;
scaleArmsJointsLimits  = 1;
scaleLegsJointsLimits  = 1;

armsJointsLimit        = scaleArmsJointsLimits *  [-170, 15;   10, 160; -45, 75; 0,  75];

if Config.experimentCase == 1 % flying

    torsoJointsLimit   = scaleTorsoJointsLimits * [-40, 40];
    legsJointsLimit    = scaleLegsJointsLimits * [-65, 65; -70, 5; -40, 30; -20, 20];

elseif Config.experimentCase == 0 % hovering
    
    torsoJointsLimit   = scaleTorsoJointsLimits * [-15,  40;  -40, 40]; %[-15,  40;  -20, 20;  -40, 40];
    legsJointsLimit    = scaleLegsJointsLimits * [-35,  85; 5, 90; -65, 65; -70, 5; -40, 30; -20, 20];
end

Config.sat.jointPositionLimits = [torsoJointsLimit; % torso
    armsJointsLimit;           % larm
    armsJointsLimit;           % rarm
    legsJointsLimit;           % lleg
    legsJointsLimit] * pi/180; % rleg

%% FRAMES AND PORTS

% Robot frames list
Frames.BASE_LINK        = 'root_link';
Frames.JET1_FRAME       = 'l_arm_jet_turbine';
Frames.JET2_FRAME       = 'r_arm_jet_turbine';
Frames.JET3_FRAME       = 'chest_l_jet_turbine';
Frames.JET4_FRAME       = 'chest_r_jet_turbine';
Frames.COM_FRAME        = 'com';
Frames.LFOOT_FRAME      = 'l_sole';
Frames.RFOOT_FRAME      = 'r_sole';
Frames.LLEG_FT_FRAME    = 'l_leg_ft_sensor';
Frames.RLEG_FT_FRAME    = 'r_leg_ft_sensor';
Frames.WAIST_IMU        = 'root_link_imu_acc';

% Robot ports list
Ports.WRENCH_LEFT_FOOT  = '/wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o';
Ports.WRENCH_RIGHT_FOOT = '/wholeBodyDynamics/right_foot_front/cartesianEndEffectorWrench:o';
Ports.JOYSTICK_AXIS     = '/joypadDevice/xbox/axis:o';
Ports.JOYSTICK_BUTTONS  = '/joypadDevice/xbox/buttons:o';

Ports.IMU               = ['/' WBTConfigRobot.RobotName '/inertial'];
Ports.IMU_Root          = ['/' WBTConfigRobot.RobotName '/xsens_inertial'];
Ports.NECK_POS          = ['/' WBTConfigRobot.RobotName '/head/state:o'];
Ports.BASE_STATE        = ['/' WBTConfigRobot.RobotName '/floating_base/state:o'];
Ports.BASE_STATE_new    = ['/' WBTConfigRobot.RobotName '/floating_base/stateBase:o'];
Ports.FT_LEFT_ARM       = ['/' WBTConfigRobot.RobotName '/left_arm/analog:o'];
Ports.FT_RIGHT_ARM      = ['/' WBTConfigRobot.RobotName '/right_arm/analog:o'];
Ports.FT_LEFT_LEG       = ['/' WBTConfigRobot.RobotName '/left_leg/analog:o'];
Ports.FT_RIGHT_LEG      = ['/' WBTConfigRobot.RobotName '/right_leg/analog:o'];
Ports.FT_LEFT_FOOT      = ['/' WBTConfigRobot.RobotName '/left_foot/analog:o'];
Ports.FT_RIGHT_FOOT     = ['/' WBTConfigRobot.RobotName '/right_foot/analog:o'];

% Floating base estimation
BaseEstPorts.BASE_ESTIMATOR_EKF  = '/base-estimator-ekf/floating_base/state:o';
BaseEstPorts.BASE_ESTIMATOR_V1   = '/base-estimator/floating_base/state:o';
realsense_port                   = '/t265';

% WholeBodyDynamics for thrust estimation
ExtWBDPorts.WRENCH_TORSO     = '/wholeBodyDynamics-thrEst/torso/cartesianEndEffectorWrench:o';
ExtWBDPorts.WRENCH_LEFT_ARM  = '/wholeBodyDynamics-thrEst/left_arm/cartesianEndEffectorWrench:o';
ExtWBDPorts.WRENCH_RIGHT_ARM = '/wholeBodyDynamics-thrEst/right_arm/cartesianEndEffectorWrench:o';
