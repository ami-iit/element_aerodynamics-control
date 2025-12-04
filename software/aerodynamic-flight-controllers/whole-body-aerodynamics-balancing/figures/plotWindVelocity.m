close all;
clear all;
clc;

%% Define settings
varName       = 'windSpeed_SCOPE';
yVarLabel     = '$v_w$ [m/s]';

expType       = 2; 

yLimits        = [-16 2];
legendLocation = 'best';

figNamePrefix = varName(1:end-6);

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

y1 = squeeze(out.(varName).signals.values(:,1));
y2 = squeeze(out.(varName).signals.values(:,2));
y3 = squeeze(out.(varName).signals.values(:,3));

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
saveF([figNamePrefix,'.pdf'],[20 16])
