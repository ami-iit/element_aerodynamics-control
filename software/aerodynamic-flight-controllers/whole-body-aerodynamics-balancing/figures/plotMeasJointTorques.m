close all;
clear all;
clc;

%% Define settings
varName       = 'jointTorqueMeas_SCOPE';
yVarLabel     = '$\tau_{meas}$ [N$\cdot$m]';

expType       = 2;     % 'front': 1 | 'lateral': 2
jointGroup    = 4;

yLimits        = [-10 30];
legendLocation = 'nw';

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

y1 = squeeze(out.(varName).signals(jointGroup).values(:,1));
y2 = squeeze(out.(varName).signals(jointGroup).values(:,2));
y3 = squeeze(out.(varName).signals(jointGroup).values(:,3));

if jointGroup ~= 1
    y4 = squeeze(out.(varName).signals(jointGroup).values(:,4));

    if jointGroup == 4 || jointGroup == 5
        y5 = squeeze(out.(varName).signals(jointGroup).values(:,5));
        y6 = squeeze(out.(varName).signals(jointGroup).values(:,6));
    end
end

%% plot
legendNames = {{'torso pitch','torso roll','torso yaw'}, ...
               {'left arm pitch','left arm roll','left arm yaw', 'left elbow'}, ...
               {'right arm pitch','right arm roll','right arm yaw', 'right elbow'}, ...
               {'left leg pitch','left leg roll','left leg yaw', 'left knee', 'left ankle pitch', 'left ankle roll'}, ...
               {'right leg pitch','right leg roll','right leg yaw', 'right knee', 'right ankle pitch', 'right ankle roll'}};

fig = figure();
fig.Position = [200 300 900 600];
ax = gca;
hold on
plot(x,y1,x,y2,x,y3,'LineWidth',2);
if jointGroup ~= 1 
    plot(x,y4,'LineWidth',2); 
    if jointGroup == 4 || jointGroup == 5 
        plot(x,y5,x,y6,'LineWidth',2); 
    end
end
grid on;

ax.FontSize = 20;
xlim([min(x) max(x)])
if exist('yLimits','var'), ylim(yLimits); end

xlabel('t [s]','Interpreter','latex','FontSize',30);
ylabel(yVarLabel,'Interpreter','latex','FontSize',30);
legend(legendNames{jointGroup},'FontSize',24,'Location',legendLocation);

% save image
saveF([figNamePrefix,'.pdf'],[20 16])
