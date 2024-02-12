% VISUALIZEOPTIMALCONFIGURATION runs an algorithm to show the optimal configuration generated
%                               by the algorithm runLocalChangesOptimization.m
%
% Author: Gabriele Nava (gabriele.nava@iit.it)
% Modified by: Fabio Di Natale
% Genova, Oct 2022
%

clear variables
close all
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                          TUNABLE PARAMETERS                           %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set robot name
robotName = 'iRonCub-Mk1';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                          SETUP ROBOT DATA                             %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add path to local functions
addpath(genpath('./src'))

% Initialize robot and optimization parameters
run(['../app/robots/',robotName,'/configRobot.m']);
run('./init/initOptimization.m');

% Reading the configurations
optimizedConfig = readtable(['./optimal_generated_csv/',robotName,'_jointConfigurations.csv']);

% Assign configuration joint positions
jointPosMatrix = table2array(optimizedConfig(:,2:end));

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                       VISUALIZE CONFIGURATIONS                        %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Preparing robot pose visualization...')

jointConfigNumber = size(jointPosMatrix,1);

for i = 1 : jointConfigNumber

% Visualize the configuration

% figure window style. Default: 'normal'
figureWindowStyle = 'docked';
set(0,'DefaultFigureWindowStyle',figureWindowStyle)
   
% Optimal pose visualization
visualizeRobot(jointPosMatrix(i,:)*(pi/180), KinDynModel, Config)
    
end

% Remove local paths
rmpath(genpath('./src'))
