% Description: This script generates different sets of all the joint angles 
% following an assigned Gaussian distribution. It is possible to fix any
% joint position to a desired value (locked joint position in the config
% file). The code has been implemented to work for iRonCub-Mk1 (installing 
% https://github.com/ami-iit/ironcub-mk1-software) and iRonCub-Mk3 
% (installing https://github.com/ami-iit/component_ironcub)
%
% Authors: Antonello Paolino, Fabio Di Natale, Gabriele Nava
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

% Select the number of configurations to be generated
jointConfigNumber = 10;

% Set the Gaussian distribution parameters
beta  = 0.7; % truncate gaussian to this percentage of joint range

% Select the joints to be blocked (0: free, 1:locked)
torsoLockedJoints = [0, 0, 0];
leftArmLockedJoints = [0, 0, 0, 0];
rightArmLockedJoints = [0, 0, 0, 0];
leftLegLockedJoints = [0, 0, 0, 0, 1, 1];
rightLegLockedJoints = [0, 0, 0, 0, 1, 1];

% Build locked joints logic vector
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

% Assign locked joint positions
lockedJointConfig = Config.robot.lockedJointConfig;

% Assign joints number variable
jointsNumber = Config.robot.ndof;

% Assign vectors of min and max joint limits
minJointLimits  = Config.robot.minJointLimits;
maxJointLimits  = Config.robot.maxJointLimits;

% Compute vectors of means and scaled min and max joint limits
sigmas  = (maxJointLimits - minJointLimits) / 6; % 6-sigma interval
meanJointValues = (minJointLimits + maxJointLimits)/2;
scaledMinJointLimits = meanJointValues - beta * (meanJointValues - minJointLimits);
scaledMaxJointLimits = meanJointValues + beta * (maxJointLimits - meanJointValues);

% Assign home configuration cell
homeConfigNames = Config.robot.homeConfigNames;

% Assign home positions matrix
homePosMatrix = Config.robot.homePosMatrix;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                     GENERATE RANDOM CONFIGURATIONS                    %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize new generated configuration names
newConfigNames = cell(jointConfigNumber,1);

% Assign new generated configuration names
for configIndex = 1 : jointConfigNumber
    newConfigNames{configIndex} = ['Config',num2str(configIndex)];
end

% Initialize joint configuration values matrix
newJointPosMatrix = zeros(jointConfigNumber,jointsNumber);

% Assign new joint position matrix values
for jointIndex = 1 : jointsNumber
    
    if lockedJoints(jointIndex)
        newJointPosMatrix(:,jointIndex) = ones(jointConfigNumber,1) * lockedJointConfig(jointIndex);
    else
        % Get mean joint value and scaled joint limits for gaussian 
        % distribution truncation
        meanJointValue      = meanJointValues(jointIndex);
        scaledMinJointLimit = scaledMinJointLimits(jointIndex);
        scaledMaxJointLimit = scaledMaxJointLimits(jointIndex);
        sigma = sigmas(jointIndex);

        % Create a Gaussian distribution N(mean,sigma), truncated at 
        % beta*limits
        gaussianDistribution = truncate(makedist('Normal', meanJointValue, sigma), scaledMinJointLimit, scaledMaxJointLimit);

        % Generating joint values from the truncated Gaussian distribution
        newJointPosMatrix(:,jointIndex) = random(gaussianDistribution, [jointConfigNumber, 1]);

        % Generating random numbers from Uniform distribution (legacy)
        % minJointLimit = minJointLimits(jointIndex);
        % maxJointLimit = maxJointLimits(jointIndex);
        % newJointPosMatrix(:,jointIndex) = minJointLimit + ( maxJointLimit - minJointLimit ) * rand(jointConfigNumber,1);
    end

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                         PLOT THE GENERATED DATA                       %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot torso joints data
figure('Name','torso joints [deg]')
tiledlayout(2,2)
for jointIndex = 1 : 3
    plotTile(jointIndex,minJointLimits,maxJointLimits,...
             newJointPosMatrix,scaledMinJointLimits,scaledMaxJointLimits);
end

% Plot left arm joints data
figure('Name','left arm joints [deg]')
tiledlayout(2,2)
for jointIndex = 4 : 7
    plotTile(jointIndex,minJointLimits,maxJointLimits,...
             newJointPosMatrix,scaledMinJointLimits,scaledMaxJointLimits);
end

% Plot right arm joints data
figure('Name','right arm joints [deg]')
tiledlayout(2,2)
for jointIndex = 8 : 11
    plotTile(jointIndex,minJointLimits,maxJointLimits,...
             newJointPosMatrix,scaledMinJointLimits,scaledMaxJointLimits);
end

% Plot left leg joints data
figure('Name','left leg joints [deg]')
tiledlayout(2,3)
for jointIndex = 12 : 17
    plotTile(jointIndex,minJointLimits,maxJointLimits,...
             newJointPosMatrix,scaledMinJointLimits,scaledMaxJointLimits);
end

% Plot right leg joints data
figure('Name','right leg joints [deg]')
tiledlayout(2,3)
for jointIndex = 18 : 23
    plotTile(jointIndex,minJointLimits,maxJointLimits,...
             newJointPosMatrix,scaledMinJointLimits,scaledMaxJointLimits);
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

function [] = plotTile(jointIndex, minJointLimits, maxJointLimits, ...
                       newJointPosMatrix, scaledMinJointLimits, scaledMaxJointLimits)
    
    minJointLimit = minJointLimits(jointIndex);
    maxJointLimit = maxJointLimits(jointIndex);

    scaledMinJointLimit = scaledMinJointLimits(jointIndex);
    scaledMaxJointLimit = scaledMaxJointLimits(jointIndex);
    
    nexttile
    hold on;
    grid on;
    xlim([minJointLimit maxJointLimit]);
    
    plot([minJointLimit maxJointLimit], [0 0], 'r-');
    plot([scaledMinJointLimit scaledMinJointLimit],[-1 1],'g-');
    plot([scaledMaxJointLimit scaledMaxJointLimit],[-1 1],'g-');

    scatter(newJointPosMatrix(:,jointIndex), ...
            newJointPosMatrix(:,jointIndex)*0, 12, 'k', 'filled');

end
