% Description: This script visualizes the optimized robot configurations 
% generated with the script runLocalChangesOptimization. The code has been 
% implemented to work for iRonCub-Mk1 
% (installing https://github.com/ami-iit/ironcub-mk1-software) and 
% iRonCub-Mk3 (installing https://github.com/ami-iit/component_ironcub)
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

% Set figure window style. Default: 'normal'
figureWindowStyle = 'docked';
set(0,'DefaultFigureWindowStyle',figureWindowStyle)
   
% Optimal pose visualization
visualizeRobot(jointPosMatrix(i,:)*(pi/180), KinDynModel, Config)
    
end

% Restore figure window style
figureWindowStyle = 'normal';
set(0,'DefaultFigureWindowStyle',figureWindowStyle)

% Remove local paths
rmpath(genpath('./src'))
