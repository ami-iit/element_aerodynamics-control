% This script generates different random set of all the joint angles for 
% iRonCub MK1 robot. It's possible to set any joint to be always fixed to
% the same value equal to the home position.
%
% Author: Fabio Di Natale
% Modified by: Gabriele Nava, Antonello Paolino
%
% March 2022.
%

clear variables
close all
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                          TUNABLE PARAMETERS                           %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set robot name
robotName = 'iRonCub-Mk1';

% Select the number of configurations to be generated
jointConfigNumber = 10;

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

% Run robot configuration script
run(['../app/robots/',robotName,'/configRobot.m']);

% Assign joints number variable
jointsNumber = Config.robot.ndof;

% Assign vectors of min and max joint limits
minJointLimits  = Config.robot.minJointLimits;
maxJointLimits  = Config.robot.maxJointLimits;

% Assign home configuration cell
homeConfigNames = Config.robot.homeConfigNames;

% Assign home positions matrix
homePosMatrix = Config.robot.homePosMatrix;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                     GENERATE RANDOM CONFIGURATIONS                    %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

newConfigNames = cell(jointConfigNumber,1);

% Assign new generated configuration names
for configIndex = 1 : jointConfigNumber
    newConfigNames{configIndex} = ['Config',num2str(configIndex)];
end

% Initialize joint configuration values matrix
newJointPosMatrix = zeros(jointConfigNumber,jointsNumber);

% Assign new joint position matrix values
for jointIndex = 1 : jointsNumber

    minJointLimit = minJointLimits(jointIndex);
    maxJointLimit = maxJointLimits(jointIndex);
    
    if lockedJoints(jointIndex)
        newJointPosMatrix(:,jointIndex) = ones(jointConfigNumber,1)*Config.robot.homePosHovering(jointIndex);
    else
        newJointPosMatrix(:,jointIndex) = minJointLimit + ( maxJointLimit - minJointLimit ) * rand(jointConfigNumber,1);
    end

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                         PLOT THE GENERATED DATA                       %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot torso joints data
figure('Name','torso joints [deg]')
tiledlayout(2,2)
for jointIndex = 1 : 3
    plotTile(jointIndex,minJointLimits,maxJointLimits,newJointPosMatrix);
end

% Plot left arm joints data
figure('Name','left arm joints [deg]')
tiledlayout(2,2)
for jointIndex = 4 : 7
    plotTile(jointIndex,minJointLimits,maxJointLimits,newJointPosMatrix);
end

% Plot right arm joints data
figure('Name','right arm joints [deg]')
tiledlayout(2,2)
for jointIndex = 8 : 11
    plotTile(jointIndex,minJointLimits,maxJointLimits,newJointPosMatrix);
end

% Plot left leg joints data
figure('Name','left leg joints [deg]')
tiledlayout(2,3)
for jointIndex = 12 : 17
    plotTile(jointIndex,minJointLimits,maxJointLimits,newJointPosMatrix);
end

% Plot right leg joints data
figure('Name','right leg joints [deg]')
tiledlayout(2,3)
for jointIndex = 18 : 23
    plotTile(jointIndex,minJointLimits,maxJointLimits,newJointPosMatrix);
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                  GENERATE FULL JOINT POSITION MATRIX                  %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Assign full configuration names cell
fullConfigNames = [homeConfigNames; newConfigNames];

% Generate the matrix adding the symmetric configurations in the start
fullJointPosMatrix = [homePosMatrix; newJointPosMatrix];

% Round data to .0
fullJointPosMatrix = round(fullJointPosMatrix,1);

% Set save directory and file names
saveFileName = [robotName,'_jointConfigurations.csv'];
saveDirPath = './generated_csv/';
if ~exist(saveDirPath,'dir'), mkdir(saveDirPath); end

% Export .csv
writetable(cell2table([fullConfigNames num2cell(fullJointPosMatrix)]), ...
           [saveDirPath,saveFileName],'writevariablenames',0);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                             PLOT FUNCTION                             %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = plotTile(jointIndex,minJointLimits,maxJointLimits,newJointPosMatrix)
    
    minJointLimit = minJointLimits(jointIndex);
    maxJointLimit = maxJointLimits(jointIndex);
    
    nexttile
    hold on;
    grid on;
    xlim([minJointLimit maxJointLimit]);
    
    plot([minJointLimit maxJointLimit], [0 0], 'r-');

    scatter(newJointPosMatrix(:,jointIndex), ...
            newJointPosMatrix(:,jointIndex)*0, 12, 'k', 'filled');

end
