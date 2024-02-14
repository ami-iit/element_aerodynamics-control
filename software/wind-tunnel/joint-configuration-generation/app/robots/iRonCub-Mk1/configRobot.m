%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON ROBOT CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Robot general info
% General robot model information
Config.robot.ndof = 23;

% Joints limits list. Limits are scaled by a safety range (smaller than the
% normal joints limit range)

Config.robot.scaleTorsoJointsLimits = 0.7;
Config.robot.scaleArmsJointsLimits  = 0.8;
Config.robot.scaleLegsJointsLimits  = 0.5;

Config.robot.torsoJointsLimit = Config.robot.scaleTorsoJointsLimits * [-20, 70; -30, 30; -50, 50];
Config.robot.armsJointsLimit  = Config.robot.scaleArmsJointsLimits * [-90, 10; 0, 160; -35, 80; 15, 105];
Config.robot.legsJointsLimit  = Config.robot.scaleLegsJointsLimits *[-35, 80; -15, 90; -70, 70; -100, 0; -30, 30; -20, 20];

% Assign joint limits
% Create vectors of min and max joint limits
Config.robot.minJointLimits  = [Config.robot.torsoJointsLimit(:,1); ...
                                Config.robot.armsJointsLimit(:,1); ...
                                Config.robot.armsJointsLimit(:,1); ...
                                Config.robot.legsJointsLimit(:,1); ...
                                Config.robot.legsJointsLimit(:,1)];
Config.robot.maxJointLimits  = [Config.robot.torsoJointsLimit(:,2); ...
                                Config.robot.armsJointsLimit(:,2); ...
                                Config.robot.armsJointsLimit(:,2); ...
                                Config.robot.legsJointsLimit(:,2); ...
                                Config.robot.legsJointsLimit(:,2)];

%% Set locked joint positions
Config.robot.lockedJointConfig = [zeros(1,3), zeros(1,4), zeros(1,4), [0 10 0 zeros(1,3)], [0 10 0 zeros(1,3)]];

%% Set wind tunnel home positions
% Initialize home configuration cell
Config.robot.homeConfigNames = cell(4,1);

% Assign home configuration names
Config.robot.homeConfigNames{1} = 'hovering';
Config.robot.homeConfigNames{2} = 'flight30';
Config.robot.homeConfigNames{3} = 'flight50';
Config.robot.homeConfigNames{4} = 'flight60';

% Default Home Positions
Config.robot.homePosHovering = [zeros(1,3), [-10   25   40   15],   [-10   25   40   15],   [0 10 7 zeros(1,3)], [0 10 7 zeros(1,3)]];
Config.robot.homePosFlight30 = [zeros(1,3), [-40.7 11.3 26.5 58.3], [-40.7 11.3 26.5 58.3], [0 10 7 zeros(1,3)], [0 10 7 zeros(1,3)]];
Config.robot.homePosFlight50 = [zeros(1,3), [-31.3 19   26.3 45.3], [-31.3 19   26.3 45.3], [0 10 7 zeros(1,3)], [0 10 7 zeros(1,3)]];
Config.robot.homePosFlight60 = [zeros(1,3), [-25   24   30   35],   [-25   24   30   35],   [0 10 7 zeros(1,3)], [0 10 7 zeros(1,3)]];

% Set home positions matrix
Config.robot.homePosMatrix = [Config.robot.homePosHovering; ...
                              Config.robot.homePosFlight30; ...
                              Config.robot.homePosFlight50; ...
                              Config.robot.homePosFlight60];

%% Set robot model info

% Specify the list of joints that are going to be considered in the reduced model
Config.robot.jointList = {'torso_pitch','torso_roll','torso_yaw', ...
                          'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                          'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                          'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                          'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
        
% Select the link that will be used as base link
Config.robot.baseLinkName = 'root_link';

% Model name and path (hard-coded for the moment)
Config.robot.component_path = getenv('IRONCUB_SOFTWARE_SOURCE_DIR');
Config.robot.modelName      = 'model_stl.urdf';
Config.robot.meshesPath     = [Config.robot.component_path '\models'];
Config.robot.modelPath      = [Config.robot.component_path '\models\iRonCub-Mk1\iRonCub\robots\iRonCub-Mk1\'];
Config.robot.DEBUG          = false;

% Set initial base pose w.r.t. the world frame the gravity vector
Config.robot.w_H_b_init = eye(4); 
Config.robot.gravityAcc = [0;0;-9.81];

% Load the reduced model
KinDynModel = iDynTreeWrappers.loadReducedModel(Config.robot.jointList, Config.robot.baseLinkName, ...
                                                Config.robot.modelPath, Config.robot.modelName, Config.robot.DEBUG); 

%% Set collision parameters

% Turn on to verify that the current robot configuration does not contain
% self collisions. WARNING: the solver may find intermediate solutions
% which do not respect constraints, it is fine.
Config.DEBUG_COLLISIONS = false;

% List of links with collisions
Config.collisions.framesList = {'l_arm_jet_turbine', 'r_arm_jet_turbine', ...
                                'chest_l_jet_turbine', 'chest_r_jet_turbine' , ...
                                'l_upper_arm', 'r_upper_arm', ...
                                'l_upper_leg', 'r_upper_leg', ...
                                'l_lower_leg', 'r_lower_leg', ...
                                'root_link', 'chest'};

%---------------------------- Arm Turbines -------------------------------%
Config.collisions.(Config.collisions.framesList{1}).centers  = [0.0  0.0  -0.03;
                                                                0.0  0.0  -0.08;
                                                                0.0  0.0  -0.14;
                                                                0.0  0.0  -0.19];

Config.collisions.(Config.collisions.framesList{2}).centers  = Config.collisions.(Config.collisions.framesList{1}).centers;

Config.collisions.(Config.collisions.framesList{1}).radiuses = [0.035; 0.05; 0.05; 0.05];

Config.collisions.(Config.collisions.framesList{2}).radiuses = Config.collisions.(Config.collisions.framesList{1}).radiuses;

%---------------------------- Jetpack Turbines ----------------------------%
Config.collisions.(Config.collisions.framesList{3}).centers  = [0.0  0.0  -0.06;
                                                                0.0  0.0  -0.12;
                                                                0.0  0.0  -0.16;
                                                                0.0  0.0  -0.225];
 
Config.collisions.(Config.collisions.framesList{4}).centers  = Config.collisions.(Config.collisions.framesList{3}).centers;

Config.collisions.(Config.collisions.framesList{3}).radiuses = [0.05; 0.06; 0.06; 0.06];

Config.collisions.(Config.collisions.framesList{4}).radiuses = Config.collisions.(Config.collisions.framesList{3}).radiuses;

%------------------------------ Upper Arms -------------------------------%
Config.collisions.(Config.collisions.framesList{5}).centers  = [-0.015  0.0  0.005;
                                                                -0.015  0.0  0.05];

Config.collisions.(Config.collisions.framesList{6}).centers  = Config.collisions.(Config.collisions.framesList{5}).centers;

Config.collisions.(Config.collisions.framesList{5}).radiuses = [0.0475; 0.0475];

Config.collisions.(Config.collisions.framesList{6}).radiuses = Config.collisions.(Config.collisions.framesList{5}).radiuses;

%------------------------------ Upper Legs -------------------------------%
Config.collisions.(Config.collisions.framesList{7}).centers  = [0.005  0.0   0.01;
                                                                0.01   0.0  -0.06];

Config.collisions.(Config.collisions.framesList{8}).centers  = Config.collisions.(Config.collisions.framesList{7}).centers;

Config.collisions.(Config.collisions.framesList{7}).radiuses = [0.06; 0.06];

Config.collisions.(Config.collisions.framesList{8}).radiuses = Config.collisions.(Config.collisions.framesList{7}).radiuses;

%------------------------------ Lower Legs -------------------------------%
Config.collisions.(Config.collisions.framesList{9}).centers   = [0.01  0.0 -0.04;
                                                                 0.0   0.0 -0.1;
                                                                 0.0   0.0 -0.14];

Config.collisions.(Config.collisions.framesList{10}).centers  = Config.collisions.(Config.collisions.framesList{9}).centers;

Config.collisions.(Config.collisions.framesList{9}).radiuses  = [0.06; 0.06; 0.06];

Config.collisions.(Config.collisions.framesList{10}).radiuses = Config.collisions.(Config.collisions.framesList{9}).radiuses;

%------------------------------ Base Link --------------------------------%
Config.collisions.(Config.collisions.framesList{11}).centers  = [0.02   0.05 -0.02;
                                                                 0.02  -0.05 -0.02;
                                                                 0.02   0.0  -0.09;
                                                                 0.02   0.0  -0.18];

Config.collisions.(Config.collisions.framesList{11}).radiuses = [0.06; 0.06; 0.065; 0.03];

%-------------------------------- Chest ----------------------------------%
Config.collisions.(Config.collisions.framesList{12}).centers  = [ 0.0    0.04  0.01;
                                                                 -0.045  0.02  -0.1;
                                                                  0.045  0.02  -0.1;
                                                                  0.0    0.1   -0.1];

Config.collisions.(Config.collisions.framesList{12}).radiuses = [0.08; 0.06; 0.06; 0.095];

