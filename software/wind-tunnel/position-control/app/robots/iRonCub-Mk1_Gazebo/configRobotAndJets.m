%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON ROBOT CONFIGURATION PARAMETERS                      %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% GENERAL ROBOT MODEL INFORMATION

Config.N_DOF         = 23;
Config.N_DOF_MATRIX  = eye(Config.N_DOF);
Config.ON_GAZEBO     = true;
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
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% Controlboards and joints list. Each joint is associated to the corresponding controlboard 
WBTConfigRobot.ControlBoardsNames     = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints       = [];
Config.numOfJointsForEachControlboard = [];

ControlBoards                                        = struct();
ControlBoards.(WBTConfigRobot.ControlBoardsNames{1}) = {'torso_pitch','torso_roll','torso_yaw'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{2}) = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{3}) = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{4}) = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
ControlBoards.(WBTConfigRobot.ControlBoardsNames{5}) = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

for n = 1:length(WBTConfigRobot.ControlBoardsNames)

    WBTConfigRobot.ControlledJoints       = [WBTConfigRobot.ControlledJoints, ControlBoards.(WBTConfigRobot.ControlBoardsNames{n})];
    Config.numOfJointsForEachControlboard = [Config.numOfJointsForEachControlboard; length(ControlBoards.(WBTConfigRobot.ControlBoardsNames{n}))];
end

%% JOINT LIMITS

% Joints limits list. Limits are scaled by a safety range (smaller than the
% normal joints limit range)
scaleTorsoJointsLimits = 0.7;
scaleArmsJointsLimits  = 1;
scaleLegsJointsLimits  = 0.75;

torsoJointsLimit       = scaleTorsoJointsLimits * [15, 45; -20, 20; -20, 20];
armsJointsLimit        = scaleArmsJointsLimits  * [-90, 10; 0, 160; -30, 80; 15, 100];
legsJointsLimit        = scaleLegsJointsLimits  * [-35, 80; 5, 90; -70, 70; -35, 0; -30, 30; -20, 20];

Config.sat.jointPositionLimits = [torsoJointsLimit;          % torso
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
                 
% Robot ports list
Ports.THROTTLE          = '/icub-jets/jets/input:i';
Ports.THRUST            = '/icub-jets/jets/thrust:o';
Ports.THRUST_DOT        = '/icub-jets/jets/d_thrust:o';
Ports.THRUST_RATIO      = '/icub-jets/jets/input:i';
Ports.WRENCH_LEFT_FOOT  = '/wholeBodyDynamics/left_foot/cartesianEndEffectorWrench:o';
Ports.WRENCH_RIGHT_FOOT = '/wholeBodyDynamics/right_foot/cartesianEndEffectorWrench:o';
Ports.BASE_STATE        = ['/' WBTConfigRobot.RobotName '/floating_base/state:o'];
Ports.FT_LEFT_ARM       = ['/' WBTConfigRobot.RobotName '/left_arm/analog:o'];
Ports.FT_RIGHT_ARM      = ['/' WBTConfigRobot.RobotName '/right_arm/analog:o'];
Ports.FT_LEFT_LEG       = ['/' WBTConfigRobot.RobotName '/left_leg/analog:o'];
Ports.FT_RIGHT_LEG      = ['/' WBTConfigRobot.RobotName '/right_leg/analog:o'];
Ports.FT_LEFT_FOOT      = ['/' WBTConfigRobot.RobotName '/left_foot/analog:o'];
Ports.FT_RIGHT_FOOT     = ['/' WBTConfigRobot.RobotName '/right_foot/analog:o'];
Ports.IMU               = ['/' WBTConfigRobot.RobotName '/inertial'];
