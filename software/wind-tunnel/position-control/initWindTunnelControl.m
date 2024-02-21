%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2018 
%  * @author: Daniele Pucci & Gabriele Nava & Antonello Paolino
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars -except sl_synch_handles
clc

%% GENERAL SIMULATION INFO

% Set path to the utility functions and to WBC library
import wbc.*
addpath(genpath('./src/'));
addpath(genpath('../matlab-functions-lib/'));

% Simulation time and delta_t [s]
Config.simulationTime                   = inf;
Config.tStep                            = 0.01;
YARP_ROBOT_NAME                         = getenv('YARP_ROBOT_NAME');

%% SIMULATION SETTINGS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select if the control signals are applied to the robot/turbines
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Config.APPLY_CONTROL                    = true;

% Save data on the workspace after the simulation
Config.SAVE_WORKSPACE                   = true;

%% WIND TUNNEL CONFIGURATIONS SETUP
% Select experiment configuration to load the joint configuration file: 
%     HOVERING = 0    FLIGHT = 1
Config.experimentCase = 1;

if Config.experimentCase == 0       % HOVERING
    joints_references_filename = "Hovering_Configurations.csv";
elseif Config.experimentCase == 1   % FLIGHT
    joints_references_filename = "Flight_Configurations.csv";
end

Config.joints_references  = readmatrix(joints_references_filename,'Range',[1 2]);
Config.joints_references  = Config.joints_references * pi/180;
Config.N_joints_reference = length(Config.joints_references(:,1));

% Extract the joints references names
ConfigGUI.joints_references_names  = readtable(joints_references_filename,'Range','A:A');
ConfigGUI.joints_references_names  = table2array(ConfigGUI.joints_references_names);

% Initialize the current state from the starting position -> -1
Config.currentState = -1;

% Fixed base simulation/experiment base initialization
Config.FIXED_BASE                       = true;
Config.w_H_b_baseFixed                  = eye(4,4);
Config.w_baseFixedTwist                 = zeros(6,1);

% Time info to change the robot configuration
Config.initialBalancingTime             = 5;
Config.tSettle                          = 4;  %[s] time for settling the joints

% Emergency stops
Config.EMERGENCY_STOP_WITH_JOINTS_LIMITS  = false;
Config.EMERGENCY_STOP_WITH_ENCODER_SPIKES = true;
Config.sat.maxJointsPositionDelta         = 15*pi/180; % [rad] 

%% ADD CONFIGURATION FILES

% Run robot-specific and controller-specific configuration parameters
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobotAndJets.m'));

%% START GUI
simulinkStaticGUI;

try
    set_param('WindTunnelControl/WIND TUNNEL CONTROLLER/WIND TUNNEL CONTROL/InputGUI','Value','-1')  
catch ME    
    warning(ME.message)
end
