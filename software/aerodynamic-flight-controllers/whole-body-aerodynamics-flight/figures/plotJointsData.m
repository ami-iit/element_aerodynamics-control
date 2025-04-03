close all;
clear all;
clc;

%% Define settings
varName       = 'jointPosErr_SCOPE';
yVarLabel     = '$\Delta s$ [deg]';

expType       = 6; 

yLimits        = [0 150];
legendLocation = 'nw';

figNamePrefix = varName(1:end-6);

%% Load mat file and define paths

if expType == 1
    expMatFile = '../experiments/2023-11-13/exp_17-27.mat';
    figurePath = './17-27/';
elseif expType == 2
    expMatFile = '../experiments/2023-11-13/exp_17-30.mat';
    figurePath = './17-30/';
elseif expType == 3
    expMatFile = '../experiments/2023-11-13/exp_17-36.mat';
    figurePath = './17-36/';
elseif expType == 4
    expMatFile = '../experiments/2023-11-13/exp_17-43.mat';
    figurePath = './17-43/';
elseif expType == 5
    expMatFile = '../experiments/2024-01-30/exp_12-40.mat';
    figurePath = './12-40/';
elseif expType == 6
    expMatFile = '../experiments/2025-04-03/exp_14-09.mat';
    figurePath = './14-09/';
elseif expType == 7
    expMatFile = '../experiments/2025-04-02/exp_15-45.mat';
    figurePath = './15-45/';
end

load(expMatFile);

%% assign variables

x = out.(varName).time;

% y1 = squeeze(out.(varName).signals(1).values(:,:));
% y2 = squeeze(out.(varName).signals(2).values(:,:));
% y3 = squeeze(out.(varName).signals(3).values(:,:));
y1 = squeeze(sum(abs(out.(varName).signals(1).values(:,:,:)),1));
y2 = squeeze(sum(abs(out.(varName).signals(2).values(:,:,:)),1));
y3 = squeeze(sum(abs(out.(varName).signals(3).values(:,:,:)),1));
y4 = squeeze(sum(abs(out.(varName).signals(4).values(:,:,:)),1));
y5 = squeeze(sum(abs(out.(varName).signals(5).values(:,:,:)),1));

%% plot
legendNames = {'torso','left-arm','right-arm','left-leg','right-leg'};

fig = figure();
fig.Position = [200 300 900 600];
ax = gca;
hold on
plot(x,y1,x,y2,x,y3,x,y4,x,y5,'LineWidth',2);
grid on;

ax.FontSize = 20;
xlim([min(x) max(x)])
if exist('yLimits','var'), ylim(yLimits); end
xlabel('t [s]','Interpreter','latex','FontSize',30);
ylabel(yVarLabel,'Interpreter','latex','FontSize',30);
legend(legendNames,'FontSize',24,'Location',legendLocation);

saveF([figNamePrefix,'.pdf'],[20 16])


