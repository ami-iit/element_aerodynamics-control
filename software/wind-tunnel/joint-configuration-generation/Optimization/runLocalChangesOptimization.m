% Description: This script runs an optimization algorithm to modify 
% different joint configurations to avoid robot self-collisions. It is 
% possible to fix any joint position to a desired value (locked joint 
% position in the config file). The code has been implemented to work for 
% iRonCub-Mk1 (installing https://github.com/ami-iit/ironcub-mk1-software) 
% and iRonCub-Mk3 (installing https://github.com/ami-iit/component_ironcub)
%
% Author: Gabriele Nava (gabriele.nava@iit.it)
% Modified by: Fabio Di Natale, Antonello Paolino
%
% Genova, February 2024.
%

clear variables
close all
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                          TUNABLE PARAMETERS                           %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set robot name: 'iRonCub-Mk1' or 'iRonCub-Mk3'
robotName = 'iRonCub-Mk3';

% Select the joints to be blocked (0: free, 1:locked)
torsoLockedJoints = [0, 0, 0];
leftArmLockedJoints = [0, 0, 0, 0];
rightArmLockedJoints = [0, 0, 0, 0];
leftLegLockedJoints = [0, 0, 0, 0, 1, 1];
rightLegLockedJoints = [0, 0, 0, 0, 1, 1];

lockedJoints = logical([torsoLockedJoints, ...
                        leftArmLockedJoints, ...
                        rightArmLockedJoints, ...
                        leftLegLockedJoints, ...
                        rightLegLockedJoints]);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                          SETUP ROBOT DATA                             %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add path to local functions
addpath(genpath('./src'))

% Run robot configuration script
run(['../app/robots/',robotName,'/configRobot.m']);

% Assign home configuration names and joint positions
homeConfigNames  = Config.robot.homeConfigNames;
homePosMatrix    = Config.robot.homePosMatrix;
homeConfigNumber = length(homeConfigNames);

% Collect the generated data in a table.
generatedConfig = readtable(['../Generation/generated_csv/',robotName,'_jointConfigurations.csv']);

% Assign new configuration names and joint positions
newConfigNames = table2array(generatedConfig((homeConfigNumber+1):end,1));
newJointPosMatrix = table2array(generatedConfig((homeConfigNumber+1):end,2:end));


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               RUN OPTIMIZATION ON NEW JOINT POSITIONS                 %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize the optimization
run('./init/initOptimization.m');

% Initialize optimized matrix
optJointPosMatrix = 0*newJointPosMatrix;
optJointConfigNumber = size(optJointPosMatrix,1);

% Run optimization on non-home positions
for i = 1 : optJointConfigNumber
    
    % Set current joint configuration angles as initial and desired values
    Config.opti.u_des = newJointPosMatrix(i,:)'*(pi/180);
    uInit = Config.opti.u_des;

    % Limit locked joints to (almost) fixed values
    Config.opti.upperBound(lockedJoints) = uInit(lockedJoints) + 1e-3;
    Config.opti.lowerBound(lockedJoints) = uInit(lockedJoints) - 1e-3;

    % Set the initial robot position
    iDynTreeWrappers.setRobotState(KinDynModel, Config.robot.w_H_b_init, Config.opti.u_des, ...
                                   zeros(6,1), zeros(Config.robot.ndof,1), Config.robot.gravityAcc);
    
    % The variables to be optimized are collected in a vector as follows:
    %
    %   u = [jointPos]
    %
    
    % Compute the nonlinear constraints function
    nonLinearConstraints = @(u) computeNonLinearConstraints(u, KinDynModel, Config);
    
    % Compute the cost function
    costFunction = @(u) computeCostFunction(u, KinDynModel, Config);

    %%%%%%%%%%%%%%%%%%%%% Run nonlinear optimization %%%%%%%%%%%%%%%%%%%%%%
    disp('[runJointsPositionOptimization]: running optimization...')
    
    tic;
    [uStar, fval, exitflag, output] = fmincon(costFunction, uInit, [], [], [], [], Config.opti.lowerBound, ...
        Config.opti.upperBound, nonLinearConstraints, Config.opti.fminconOptions);
    
    timeSim = toc;
    
    % Assign fixed joints values
    uStar(lockedJoints) = uInit(lockedJoints);
    
    % Assign optimized joint values to matrix
    optJointPosMatrix(i,:) = uStar*180/pi;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%% Display results %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Check 1: display the initial and optimized joints position
    disp(' ')
    disp('Joints position: [initial, optimized]')
    disp(num2str([uInit, uStar]*180/pi))
    
    disp(['[runJointsPositionsOptimization]: optimization exited correctly for config: ',newConfigNames{i}])
    
end

close all

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                  GENERATE FULL JOINT POSITION MATRIX                  %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Assign full configuration names cell
fullConfigNames = [homeConfigNames; newConfigNames];

% Generate the matrix adding the symmetric configurations in the start
fullJointPosMatrix = [homePosMatrix; optJointPosMatrix];

% Round data to .0
fullJointPosMatrix = round(fullJointPosMatrix,1);

% Set save directory and file names
saveFileName = [robotName,'_jointConfigurations.csv'];
saveDirPath = './optimal_generated_csv/';
if ~exist(saveDirPath,'dir'), mkdir(saveDirPath); end

% Export .csv
writetable(cell2table([fullConfigNames num2cell(fullJointPosMatrix)]), ...
           [saveDirPath,saveFileName],'writevariablenames',0);

% Remove local paths
rmpath(genpath('./src'))
