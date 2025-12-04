close all;
clear all;
clc;

%% Define settings
varName       = 'simAeroForces_SCOPE';
yVarLabel     = '$\Delta f_a$ [N]';

yLimits        = [-3 5];
legendLocation = 'best';

figNamePrefix = varName(1:end-6);

%% Load mat file and define paths

expMatFile = '../experiments/2024-01-30/exp_12-40.mat';
figurePath = './12-40/';
load(expMatFile);

%% assign variables

x = out.aeroForcesSim_SCOPE.time;

y1 = squeeze(sum(out.aeroForcesSim_SCOPE.signals.values(1,:,:),2));
y2 = squeeze(sum(out.aeroForcesSim_SCOPE.signals.values(2,:,:),2));
y3 = squeeze(sum(out.aeroForcesSim_SCOPE.signals.values(3,:,:),2));

y11 = squeeze(sum(out.aeroForcesControl_SCOPE.signals.values(1,:,:),2));
y22 = squeeze(sum(out.aeroForcesControl_SCOPE.signals.values(2,:,:),2));
y33 = squeeze(sum(out.aeroForcesControl_SCOPE.signals.values(3,:,:),2));

%% plot
legendNames = {'x','y','z'};

fig = figure();
fig.Position = [200 300 900 600];
ax = gca;
hold on
plot(x,y1-y11,x,y2-y22,x,y3-y33,'LineWidth',2); hold on;
% plot(x,y11,x,y22,x,y33,'LineWidth',2);
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


