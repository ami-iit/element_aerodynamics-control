clc
clear variables
close all

% this script is to load all the cfd data and apply linear regression
% with the proposed models to obtain the constant coefficients
% author: HUI TONG

%% LOAD DATA

% data from sheet "Beta=180" 
data1  = readmatrix("CFD_Data_Collection.xlsx",'Sheet','Beta=180','Range','E16:AC16');
alpha1 = [data1(1); data1(6); data1(11); data1(16); data1(21)]; % [deg] alpha angle
beta1  = 180*ones(5,1); % [deg] beta angle
Cd1    = [data1(4); data1(9); data1(14); data1(19); data1(24)]; % drag coefficients [fd/v^2]
Cl1    = [data1(5); data1(10); data1(15); data1(20); data1(25)]; % lift coefficients [fl/v^2]
ns1    = size(alpha1,1); % data size

% data from sheet "Beta=270"
data2  = readmatrix("CFD_Data_Collection.xlsx",'Sheet','Beta=270','Range','E16:AC16');
alpha2 = [data2(1); data2(6); data2(11); data2(16); data2(21)]; % [deg] alpha
beta2  = 270*ones(5,1); % [deg] beta
Cd2    = [data2(4); data2(9); data2(14); data2(19); data2(24)];
Cl2    = [data2(5); data2(10); data2(15); data2(20); data2(25)];
ns2    = size(alpha2,1);

% data from sheet "Alpha=90" deg
data3  = readmatrix("CFD_Data_Collection.xlsx",'Sheet','Alpha=90','Range','E16:AC16'); 
beta3  = [data3(1); data3(6); data3(11); data3(16); data3(21)];% [deg] beta 
ns3    = size(beta3,1);
alpha3 = 90*ones(ns3,1); % [deg] alpha
Cd3    = [data3(4); data3(9); data3(14); data3(19); data3(24)];
Cl3    = [data3(5); data3(10); data3(15); data3(20); data3(25)];

% expand the data from (0,pi) into domain (0,2pi)
% B3=[B3;180+B3(2:end)];
% Cd3=[Cd3;flip(Cd3(1:end-1))];
% Cl3=[Cl3;flip(Cl3(1:end-1))];

% data from sheet "Alpha=60" deg
data4  = readmatrix("CFD_Data_Collection.xlsx",'Sheet','Alpha=60','Range','E16:AC16');
beta4  = [data4(1); data4(6); data4(11); data4(16); data4(21)]; % [deg] beta
ns4    = size(beta4,1);
alpha4 = 60*ones(ns4,1); % [deg] alpha
Cd4    = [data4(4); data4(9); data4(14); data4(19); data4(24)];
Cl4    = [data4(5); data4(10); data4(15); data4(20); data4(25)];

% expand the data into domain 2pi
% B4=[B4;180+B4(2:end)];
% Cd4=[Cd4;flip(Cd4(1:end-1))];
% Cl4=[Cl4;flip(Cl4(1:end-1))];

% data from sheet "Alpha=120" deg
data5  = readmatrix("CFD_Data_Collection.xlsx",'Sheet','Alpha=120','Range','E16:AC16');
beta5  = [data5(1); data5(6); data5(11); data5(16); data5(21)]; % [deg] beta
ns5    = size(beta5,1); % size of expanded data
alpha5 = 120*ones(ns5,1); % [deg] alpha
Cd5    = [data5(4);data5(9);data5(14);data5(19);data5(24)];
Cl5    = [data5(5);data5(10);data5(15);data5(20);data5(25)];

% expand the data into domain 2pi
% B5=[B5;180+B5(2:end)];
% Cd5=[Cd5;flip(Cd5(1:end-1))];
% Cl5=[Cl5;flip(Cl5(1:end-1))];

%% collect all data
alpha = [alpha1; alpha2; alpha3; alpha4; alpha5]; % alpha angle of all data
beta  = [beta1; beta2; beta3; beta4; beta5]; % beta angle of all data 
Cd    = [Cd1; Cd2; Cd3; Cd4; Cd5]; % drag coefficients of all data  [fd/v^2]
Cl    = [Cl1; Cl2; Cl3; Cl4; Cl5]; % normal force coefficients of all data  [fl/v^2]

alpha_rad = deg2rad(alpha); % [rad]
beta_rad  = deg2rad(beta); % [rad]
ns        = size(alpha,1); % size of the data set

% tt   = linspace(0,360,100);  % for plotting estimated model
% tt_r = deg2rad(tt); % [rad]

%% Model identification for drag force coefficient

% cd = c0 + c1*sin(alpha)^2 * cos(beta)^3 + c2*cos(beta)^3 + c3*cos(beta)^2

R_matrix_Cd_regrLearnerApp = [(sin(alpha_rad).^2).*(cos(beta_rad).^3)  cos(beta_rad).^3 cos(beta_rad).^2];            % data for Regression Learner App
R_matrix_Cd                = [ones(ns,1) (sin(alpha_rad).^2).*(cos(beta_rad).^3)  cos(beta_rad).^3 cos(beta_rad).^2]; % data for linear regression through matlab script

Cd_model_coeff = R_matrix_Cd\Cd; % identified constant coefficients

%% Model identification for lift force coefficient

% cl = c4 + c5*sin(2alpha)*cos(beta)^2

R_matrix_Cl_regrLearnerApp = sin(2*alpha_rad).*(cos(beta_rad).^2);
R_matrix_Cl                = [ones(ns,1) sin(2*alpha_rad).*(cos(beta_rad).^2)];

Cl_model_coeff = R_matrix_Cl\Cl;

sprintf('The identified constant coefficients are:\n c0=%f,\n c1=%f,\n c2=%f,\n c3=%f,\n c4=%f,\n c5=%f', Cd_model_coeff(1),Cd_model_coeff(2),Cd_model_coeff(3),Cd_model_coeff(4),Cl_model_coeff(1),Cl_model_coeff(2));
