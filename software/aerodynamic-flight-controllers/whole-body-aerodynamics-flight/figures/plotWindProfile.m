close all;
clear all;
clc;

%% Define settings
varName       = 'windVelocity_SCOPE';
yVarLabel     = '$v_{w}$ [m/s]';

expType       = 5; 

yLimits        = [-12 2];
legendLocation = 'east';

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
end

load(expMatFile);

%% assign variables

x = out.(varName).time;

y1 = squeeze(out.(varName).signals.values(1,:,:));
y2 = squeeze(out.(varName).signals.values(2,:,:));
y3 = squeeze(out.(varName).signals.values(3,:,:));
% y1 = squeeze(out.(varName).signals.values(:,1));
% y2 = squeeze(out.(varName).signals.values(:,2));
% y3 = squeeze(out.(varName).signals.values(:,3));
% y4 = squeeze(out.(varName).signals.values(:,4));


%% plot
legendNames = {'x','y','z'};

fig = figure();
fig.Position = [200 300 900 600];
ax = gca;
hold on
plot(x,y1,x,y2,x,y3,'LineWidth',2);
grid on;

ax.FontSize = 20;
xlim([min(x) max(x)])
if exist('yLimits','var'), ylim(yLimits); end
xlabel('t [s]','Interpreter','latex','FontSize',30);
ylabel(yVarLabel,'Interpreter','latex','FontSize',30);
legend(legendNames,'FontSize',24,'Location',legendLocation);

% save image
figNameSuffix = '.eps';

% saveas(fig,[figurePath,figNamePrefix,figNameSuffix],'epsc')

saveF([figNamePrefix,'.pdf'],[20 16])


