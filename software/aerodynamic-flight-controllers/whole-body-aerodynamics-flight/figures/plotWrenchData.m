close all;
clear all;
clc;

%% Define settings
varName       = 'wrench_Rfoot_torque_QP_SCOPE';
yVarLabel     = '$f_{c,des}$ [N$\cdot$m]';

expType       = 1;     % 'front': 1 | 'lateral': 2
forceGroup    = 2;

% yLimits        = [-20 230];
legendLocation = 'best';

figNamePrefix = varName(1:end-5);

%% Load mat file and define paths

if expType == 1
    expMatFile = '../experiments/experiments19-Oct-2023/exp_17-32.mat';
    figurePath = './front_exp/';
elseif expType == 2
    expMatFile = '../experiments/experiments19-Oct-2023/exp_17-27.mat';
    figurePath = './lateral_exp/';
end

load(expMatFile);

%% assign variables

x = out.(varName).time;

y1 = squeeze(out.(varName).signals(forceGroup).values(:,1));
y2 = squeeze(out.(varName).signals(forceGroup).values(:,2));
y3 = squeeze(out.(varName).signals(forceGroup).values(:,3));

%% plot
legendNames = {{'x-force','y-force','z-force'}, ...
               {'x-torque','y-torque','z-torque'}};

fig = figure();
hold on
plot(x,y1,x,y2,x,y3);
grid on;
xlabel('t [s]','Interpreter','latex','FontSize',16);
xlim([min(x) max(x)])
if exist('yLimits','var'), ylim(yLimits); end
ylabel(yVarLabel,'Interpreter','latex','FontSize',16);
legend(legendNames{forceGroup},'FontSize',12,'Location',legendLocation);

% save image
if forceGroup == 1
    figNameSuffix = 'forces.eps';
elseif forceGroup == 2
    figNameSuffix = 'moments.eps';
end
saveas(fig,[figurePath,figNamePrefix,figNameSuffix],'epsc')


