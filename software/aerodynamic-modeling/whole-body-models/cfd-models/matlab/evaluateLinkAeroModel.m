% Author: Antonello Paolino
%
% August 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;
clc;

%% Initialization

% Data for the models
srcPath = '../src/';
dataFile = [srcPath,'outputParameters.mat'];
load(dataFile);
jointConfigNames = fieldnames(data);

%% Aerodynamic forces application points definitions
aeroFrameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
                  'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
                  'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};

cfdLinkNames   = {'head', 'torso', 'left_back_turbine', 'right_back_turbine', ...
                  'left_arm','left_arm_turbine','right_arm','right_arm_turbine',...
                  'root_link','left_leg_upper','left_leg_lower','right_leg_upper','right_leg_lower'};

%% identified aerodynamic models
aeroModelsCoeff = [0.373, 1.89, 1.235, 0.163, -1.75, 4.02;
                   3.96, 0.0, -0.818, 0.0, 0.0, 5.00;
                   0.941, -0.308, 0.320, 0, 0.433, 3.23;
                   0.941, -0.308, 0.320, 0, 0.433, 3.23;
                   0.113, 0.225, 0.684, 0, 0, 1.14;
                   0.520, 0.128, 0.860, 0, 0.163, 1.52;
                   0.113, 0.225, 0.684, 0, 0, 1.14;
                   0.520, 0.128, 0.860, 0, 0.163, 1.52;
                   1.54, 0, 3.46, -3.31, 0, 2.89;
                   0, -0.261, 1.52, 0, 0, 2.19;
                   0.853, -0.881, 4.24, -2.58, 0.434, 3.21;
                   0, -0.261, 1.52, 0, 0, 2.19;
                   0.853, -0.881, 4.24, -2.58, 0.434, 3.21]*1e-2;

%% Load dataset
load([srcPath,'datasetFullAeroFrame.mat']);

%% Assign link data
linkIndex = 1;
cfdLinkName = cfdLinkNames{linkIndex};
aeroFrameName = aeroFrameNames{linkIndex};

switch linkIndex
    case 3
        twinLinkIndex = 1;
    case 4
        twinLinkIndex = -1;
    case {5,6,10,11}
        twinLinkIndex = 2;
    case {7,8,12,13}
        twinLinkIndex = -2;
end

if linkIndex == 1 || linkIndex == 2 || linkIndex == 9
    linkAoAs_full = linkAoAs_matrix(:,linkIndex);
    linkSsAs_full = linkSsAs_matrix(:,linkIndex);
    linkCdAs_full = linkCdAs_matrix(:,linkIndex);
    linkClAs_full = linkClAs_matrix(:,linkIndex);
    linkCsAs_full = linkCsAs_matrix(:,linkIndex);
    linkCnAs_full = linkCnAs_matrix(:,linkIndex);
    linkCfAs_full = linkCfAs_matrix(:,linkIndex);
else
    linkAoAs_full = [linkAoAs_matrix(:,linkIndex); linkAoAs_matrix(:,linkIndex + twinLinkIndex)];
    linkSsAs_full = [linkSsAs_matrix(:,linkIndex); linkSsAs_matrix(:,linkIndex + twinLinkIndex)];
    linkCdAs_full = [linkCdAs_matrix(:,linkIndex); linkCdAs_matrix(:,linkIndex + twinLinkIndex)];
    linkClAs_full = [linkClAs_matrix(:,linkIndex); linkClAs_matrix(:,linkIndex + twinLinkIndex)];
    linkCsAs_full = [linkCsAs_matrix(:,linkIndex); linkCsAs_matrix(:,linkIndex + twinLinkIndex)];
    linkCnAs_full = [linkCnAs_matrix(:,linkIndex); linkCnAs_matrix(:,linkIndex + twinLinkIndex)];
    linkCfAs_full = [linkCfAs_matrix(:,linkIndex); linkCfAs_matrix(:,linkIndex + twinLinkIndex)];
end

linkCnAs_full = linkCnAs_full.*sign(cosd(linkAoAs_full));

%% Compute link model
W1 = @(alpha) [ones(length(alpha),1)        , ... 
               cosd(alpha)                  , ...
               sind(alpha).^2               , ...
               sind(alpha).^3               , ...
               cosd(alpha).^3               , ...
               ];

W2 = @(alpha) sind(alpha).^2.*cosd(alpha);

Cd_model = W1(linkAoAs_full)*aeroModelsCoeff(linkIndex,1:5)';
Cn_model = W2(linkAoAs_full)*aeroModelsCoeff(linkIndex,6);

%% Compute errors
delta_Cd = Cd_model - linkCdAs_full;
rmse_Cd = sqrt(mean(delta_Cd.^2));
delta_Cd_max = max(abs(delta_Cd))
NRMSE_Cd = 1/(max(linkCdAs_full)-min(linkCdAs_full)) * rmse_Cd

delta_Cn = Cn_model - linkCnAs_full;
rmse_Cn = sqrt(mean(delta_Cn.^2));
delta_Cn_max = max(abs(delta_Cn))
NRMSE_Cn = 1/(max(linkCnAs_full)-min(linkCnAs_full)) * rmse_Cn

%% Plots
figure_size = [100 200 1120 720];
font_size   = 32;

alpha_plot = transpose(linspace(0,180,1801));
Cd_lsq     = W1(alpha_plot)*transpose(aeroModelsCoeff(linkIndex,1:5));
Cn_lsq     = W2(alpha_plot)*aeroModelsCoeff(linkIndex,6);


% Plot CdA
fig = figure();
fig.Position = figure_size;
scatter(linkAoAs_full,linkCdAs_full,'DisplayName','CFD dataset'); hold on;
plot(alpha_plot,Cd_lsq,'k-','LineWidth',2,'DisplayName','model prediction'); hold on;
xlabel('$\alpha_{link}$ [deg]','Interpreter','latex','FontSize',font_size);
ylabel('$C_D A$','Interpreter','latex','FontSize',font_size);
xlim([0 180]);
% title(cfdLinkName,'Interpreter','none');
grid on;
legend;
set(gca,'fontsize', font_size);

% Plot CnA
fig = figure();
fig.Position = figure_size;
% scatter(linkAoAs_full,linkCnAs_full,[],linkSsAs_full); hold on;
scatter(linkAoAs_full,linkCnAs_full,'DisplayName','CFD dataset'); hold on;
plot(alpha_plot,Cn_lsq,'k-','LineWidth',2,'DisplayName','model prediction');
xlabel('$\alpha_{link}$ [deg]','Interpreter','latex','FontSize',font_size);
ylabel('$C_N A$','Interpreter','latex','FontSize',font_size);
xlim([0 180]);
% title(cfdLinkName,'Interpreter','none');
grid on;
legend;
set(gca,'fontsize', font_size);