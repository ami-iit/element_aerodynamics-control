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

%% Load dataset
load([srcPath,'datasetFullAeroFrame.mat']);

%% Assign link data
linkIndex = 11;
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

%% Generate single link CdA aerodynamic model
% X1 = @(alpha) [ones(length(alpha),1)        , ... 
%                cosd(alpha)                  , ... 
%                sind(alpha)                  , ... 
%                sind(alpha).^2               , ... 
%                sind(alpha).*cosd(alpha)     , ... 
%                cosd(alpha).^3               , ... 
%                sind(alpha).^3               , ... 
%                sind(alpha).^2.*cosd(alpha)  , ... 
%                ];

X1 = @(alpha) [ones(length(alpha),1)        , ... 
               cosd(alpha)                  , ...
               sind(alpha).^2               , ...
               sind(alpha).^3               , ...
               cosd(alpha).^3               , ...
               ];

Y1 = linkCdAs_full;

% Least Squares Regression
Cd_coefs = X1(linkAoAs_full)\Y1;
format long
disp(Cd_coefs)
Cd_predicted = X1(linkAoAs_full)*Cd_coefs;
mse = immse(Cd_predicted,Y1)

% Lasso Regression
[Cd_coefs_lasso, FitInfo] = lasso(X1(linkAoAs_full),Y1,'CV',10);
lassoPlot(Cd_coefs_lasso,FitInfo,'PlotType','CV');
grid on;
legend('show'); % Show legend

%% Generate single link CnA aerodynamic model
X2 = @(alpha) [...ones(length(alpha),1)        , ... 
               ...cosd(alpha)                  , ... 
               ...sind(alpha)                  , ... 
               ...sind(alpha).^2               , ... 
               ...sind(alpha).*cosd(alpha)     , ... 
               ...cosd(alpha).^3               , ... 
               ...sind(alpha).^3               , ... 
               sind(alpha).^2.*cosd(alpha)  , ... 
               ];

linkCnAs_full = linkCnAs_full.*sign(cosd(linkAoAs_full));
Y4 = linkCnAs_full;
Cn_coef = X2(linkAoAs_full)\Y4;

%% Plots

alpha_plot = transpose(linspace(0,180,1801));
Cd_lsq     = X1(alpha_plot)*Cd_coefs;
% Cd_lasso   = X1(alpha_plot)*Cd_coefs_lasso(:,59);
Cn_lsq     = X2(alpha_plot)*Cn_coef;

figure_size = [100 200 1120 720];
font_size   = 32;

% plot link CdAs vs AoA
fig = figure();
fig.Position = figure_size;
% scatter(linkAoAs_full,linkCdAs_full,[],linkSsAs_full); hold on;
scatter(linkAoAs_full,linkCdAs_full,'DisplayName','CFD dataset'); hold on;
plot(alpha_plot,Cd_lsq,'k-','LineWidth',2,'DisplayName','model prediction'); hold on;
xlabel('$\alpha_{link}$ [deg]','Interpreter','latex','FontSize',font_size);
ylabel('$C_D A$','Interpreter','latex','FontSize',font_size);
xlim([0 180]);
% title(cfdLinkName,'Interpreter','none');
grid on;
legend;
set(gca,'fontsize', font_size);
% c = colorbar;
% c.Limits = [0 180];
% c.Label.Interpreter = 'latex';
% c.Label.String = '$\beta_{link}$';
% c.Label.Position = [3, 95, 0];
% c.Label.Rotation = 0;
% c.Label.FontSize = 12;


% plot link CnAs vs AoA
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
% c = colorbar;
% c.Limits = [0 180];
% c.Label.Interpreter = 'latex';
% c.Label.String = '$\beta_{link}$';
% c.Label.Position = [3, 95, 0];
% c.Label.Rotation = 0;
% c.Label.FontSize = 12;


% plot link CdAs error vs AoA
fig = figure();
fig.Position = figure_size;
% scatter(linkAoAs_full,linkCdAs_full,[],linkSsAs_full); hold on;
scatter(linkAoAs_full,X1(linkAoAs_full)*Cd_coefs-linkCdAs_full); hold on;
xlabel('$\alpha_{link}$','Interpreter','latex','FontSize',24);
ylabel('$\Delta C_D A$','Interpreter','latex','FontSize',24);
xlim([0 180]);
% title(cfdLinkName,'Interpreter','none');
grid on;
set(gca,'fontsize', font_size);
% c = colorbar;
% c.Limits = [0 180];
% c.Label.Interpreter = 'latex';
% c.Label.String = '$\beta_{link}$';
% c.Label.Position = [3, 95, 0];
% c.Label.Rotation = 0;
% c.Label.FontSize = 12;