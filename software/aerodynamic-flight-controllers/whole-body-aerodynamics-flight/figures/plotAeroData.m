close all;
clear all;
clc;

%% Define settings
varName1       = 'aeroForcesSim_SCOPE';
varName2       = 'centroidal_aerodynamic_force_kf';
% varName2       = 'aeroForcesControl_SCOPE';
yVarLabel     = '$f_a$ [N]';

expType       = 9   ; 

yLimits        = [-16 5];
legendLocation = 'best';

figNamePrefix = 'aeroForce';

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
elseif expType == 8
    expMatFile = '../experiments/2025-04-03/exp_16-13.mat';
    figurePath = './16-13/';
elseif expType == 9
    expMatFile = '../experiments/2025-04-03/exp_17-22.mat';
    figurePath = './17-22/';
end

load(expMatFile);

%% assign variables

x = out.(varName1).time;

y1 = squeeze(sum(out.(varName1).signals.values(1,:,:),2));
y2 = squeeze(sum(out.(varName1).signals.values(2,:,:),2));
y3 = squeeze(sum(out.(varName1).signals.values(3,:,:),2));

y4 = out.(varName2).signals.values(:,1);
y5 = out.(varName2).signals.values(:,2);
y6 = out.(varName2).signals.values(:,3);

% y4 = squeeze(sum(out.(varName2).signals.values(1,:,:),2));
% y5 = squeeze(sum(out.(varName2).signals.values(2,:,:),2));
% y6 = squeeze(sum(out.(varName2).signals.values(3,:,:),2));

% y1 = squeeze(sum(abs(out.(varName1).signals(1).values(:,:,:)),1));
% y2 = squeeze(sum(abs(out.(varName1).signals(2).values(:,:,:)),1));
% y3 = squeeze(sum(abs(out.(varName1).signals(3).values(:,:,:)),1));
% y4 = squeeze(sum(abs(out.(varName1).signals(4).values(:,:,:)),1));
% y5 = squeeze(sum(abs(out.(varName1).signals(5).values(:,:,:)),1));

%% plot
% legendNames = {'sim-x','sim-y','sim-z','ctrl-x','ctrl-y','ctrl-z'};
legendNames = {'sim-x','sim-y','sim-z','kf-x','kf-y','kf-z'};

fig = figure();
fig.Position = [200 300 900 600];
ax = gca;
hold on
plot(x,y1,'b-',x,y2,'r-',x,y3,'g-',x,y4,'b--',x,y5,'r--',x,y6,'g--','LineWidth',2);
grid on;

ax.FontSize = 20;
xlim([min(x) max(x)])
if exist('yLimits','var'), ylim(yLimits); end
xlabel('t [s]','Interpreter','latex','FontSize',30);
ylabel(yVarLabel,'Interpreter','latex','FontSize',30);
legend(legendNames,'FontSize',24,'Location',legendLocation);

saveF([figNamePrefix,'.pdf'],[20 16])


