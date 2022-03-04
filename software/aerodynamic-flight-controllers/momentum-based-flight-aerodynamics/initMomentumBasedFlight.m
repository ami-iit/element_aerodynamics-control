%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2018 
%  * @author: Daniele Pucci & Gabriele Nava
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

% TRUE if one wants to run the runSimulationsICRA script for ICRA 2022
% simulations for the paper
Config_ICRA.RUN_SIMULATIONS_ICRA_2022 = true;

if ~Config_ICRA.RUN_SIMULATIONS_ICRA_2022
    
    clc
    clearvars -except Config_ICRA
else
    clearvars -except Config_ICRA i j
end

close all

%% GENERAL SIMULATION INFO
robotName = 'iRonCub-Mk1_1';
setenv('YARP_ROBOT_NAME', robotName)

% Set path to the utility functions and to WBC library
import wbc.*
addpath(genpath('./src/'));
addpath('../controlAndDataGui/');

if ~Config_ICRA.RUN_SIMULATIONS_ICRA_2022

    % Select the trajectory type and simulation time
    chosenSim = menu_customized('Select trajectory type','iRonCub Control GUI','Scenario 1: Hovering','Scenario 2: High-speed Flight');
else
    chosenSim = Config_ICRA.chosenSim;
end

if chosenSim==1 

    Config.USE_NATIVE_GUI        = true;
    Config.high_speed_trajectory = false;
    Config.simulationTime        = inf;
    Config.wind_gust_ramp        = 10; % [m/s]
    Config.wind_gust_cosine      = 10; % [m/s]
    Config.t_init_wind_ramp      = 5; % [s]
    Config.wind_direction        = [-1; 0; 0]; % [x y z]
    
elseif chosenSim==2
    
    Config.USE_NATIVE_GUI        = false;
    Config.high_speed_trajectory = false;
    Config.simulationTime        = 30;
    Config.wind_gust_ramp        = 10; % [m/s]
    Config.wind_gust_cosine      = 10; % [m/s]
    Config.t_init_wind_ramp      = 5; % [s]
    Config.wind_direction        = [-1; 0; 0]; % [x y z]
    
elseif chosenSim==3
    
    Config.USE_NATIVE_GUI        = false;
    Config.high_speed_trajectory = true;
    Config.simulationTime        = 45;
    Config.wind_gust_ramp        = 15; % [m/s]
    Config.wind_gust_cosine      = 15; % [m/s]
    Config.t_init_wind_ramp      = 10; % [s]
    Config.wind_direction        = [0; -1; 0]; % overwrite wind direction
end

% Settings to generate a wind profile during the simulation
Config.constant_wind       = 3; % [m/s]

% generate a wind gust with ramp function
Config.t_slope_wind_ramp   = 5;  % [s]
Config.t_plateau_wind_ramp = 2.5; % [s]
totalTimeRamp              = Config.t_init_wind_ramp + 2*Config.t_slope_wind_ramp + Config.t_plateau_wind_ramp;

% generate a wind gust with cosine function
wind_gust_cosine_after_ramp = true;
Config.t_init_wind_cosine   = 5 + totalTimeRamp * wind_gust_cosine_after_ramp; % [s]
Config.delta_t_wind_cosine  = 5; % [s]

% If true, the aerodynamic force is used as feedforward in the controller
% with a different model of the aerodynamics if selected, accounting for 
% the errors in the sensors data
if ~Config_ICRA.RUN_SIMULATIONS_ICRA_2022

    Config.use_aerodynamics_forces_feedback = false;
else
    Config.use_aerodynamics_forces_feedback = Config_ICRA.use_aerodynamics_forces_feedback;
end
Config.controller_uses_real_aerodynamics = false;

Config.controller_sensors_calib_error = -0.10;
Config.controller_sensors_noise       =  0.05;  

% If true, gain scheduling is used to enforce controller robustness under 
% the presence of wind. Applied on CoM position and velocity gains
if ~Config_ICRA.RUN_SIMULATIONS_ICRA_2022
    
    Config.use_gain_scheduling = false;
else
    Config.use_gain_scheduling = Config_ICRA.use_gain_scheduling;
end
Config.settlingTime_gainScheduling = 0.25;
Config.gains_scaling_factor        = 2.0;

% Parameters for high speed trajectory planner
Config.A_p1     = 0.6;    % acc_max for going up
Config.A_p2     = 3;      % acc_max for going forward along +x axis
Config.f_p1     = 1/5;    % frequency of going up
Config.f_p2     = 1/10;   % frequency of going forward while accelerating
Config.f_p3     = 1/10;   % frequency of going forward while decelerating
Config.Ts_const = 40;     % constant velocity flying time

%% SIMULATION SETTINGS

Config.tStep                     = 0.01;
jets_config.use_jet_dyn          = false;

% Visualizer
confVisualizer.visualizeRobot    = true;
confVisualizer.visualizeJets     = false;

% Control type:
%
% Default controller => MOMENTUM BASED CONTROL WITH LYAPUNOV STABILITY (IEEE-RAL)
% If USE_ATTITUDE_CONTROL = true => LINEAR MOMENTUM AND ATTITUDE CONTROL (IEEE-HUMANOIDS)
%
Config.USE_ATTITUDE_CONTROL      = true;
Config.Gain_scheduling           = false;

% If Config.INCLUDE_THRUST_LIMITS and/or Config.INCLUDE_JOINTS_LIMITS are
% set to true, the thrusts limits and/or the joints limits are included in
% the control algorithm (as QP constraints)
Config.INCLUDE_THRUST_LIMITS         = true;
Config.INCLUDE_JOINTS_LIMITS         = true;

% Activate visualization and data collection
Config.SCOPE_JOINTS                  = true;
Config.SCOPE_QP                      = true;
Config.SCOPE_COM                     = true;
Config.SCOPE_BASE                    = true;
Config.SCOPE_MOMENTUM                = true;
Config.SCOPE_JETS                    = true;
Config.SCOPES_WRENCHES               = true;
Config.SCOPE_GAINS_AND_STATE_MACHINE = true;

% Save data on the workspace after the simulation
Config.SAVE_WORKSPACE                = true;

%% ADD CONFIGURATION FILES

% Run robot-specific and controller-specific configuration parameters
run(strcat('app/robots/',robotName,'/configRobot.m')); 
run(strcat('app/robots/',robotName,'/gainsAndParameters.m'));
run(strcat('app/robots/',robotName,'/configJets.m'));
run(strcat('app/robots/',robotName,'/initVisualizer.m'));
run(strcat('app/robots/',robotName,'/configAerodynamics.m'));

% open the native GUI for control (if no joystick is present)
if Config.USE_NATIVE_GUI
    ironcubControlGui;
end

%% Init simulator core physics paramaters
physics_config.GRAVITY_ACC = [0;0;9.81];
physics_config.TIME_STEP   = Config.tStep;
