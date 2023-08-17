% Author: Antonello Paolino
%
% August 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;
clc;

%% Initialization

% Data for the models
dataPath = './data/';
dataFile = [dataPath,'outputParameters.mat'];
load(dataFile);
jointConfigNames = fieldnames(data);

%% Aerodynamic forces application points definitions
aeroFrameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
                  'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
                  'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};

cfdLinkNames   = {'head', 'torso', 'left_back_turbine', 'right_back_turbine', ...
                  'left_arm','left_arm_turbine','right_arm','right_arm_turbine',...
                  'root_link','left_leg_upper','left_leg_lower','right_leg_upper','right_leg_lower'};

load('./src/aeroFrameTransforms.mat');

%% data for iDynTreeWrappers
componentPath  = getenv('IRONCUB_COMPONENT_SOURCE_DIR');
modelPath      = [componentPath,'/models/iRonCub-Mk1/iRonCub/robots/iRonCub-Mk1_Gazebo/'];
fileName       = 'model_stl.urdf';
meshFilePrefix = [componentPath,'/models'];
jointNames     = {'torso_pitch','torso_roll','torso_yaw', 'l_shoulder_pitch', 'l_shoulder_roll','l_shoulder_yaw', ...
                  'l_elbow', 'r_shoulder_pitch', 'r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                  'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw','l_knee','r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee'};

jointVel = zeros(23,1);
baseVel  = zeros(6,1);
gravAcc  = [0; 0; 9.81];
basePose = eye(4);  % alpha=90 and beta=0


linkIndex = 1;

cfdLinkName = cfdLinkNames{linkIndex};
aeroFrameName = aeroFrameNames{linkIndex};

if matches(aeroFrameName, {'head','chest','root_link'})
    frameAxis  = [0; 1; 0];
    normalAxis = [1; 0; 0];
else
    frameAxis  = [0; 0; 1];
    normalAxis = [1; 0; 0];
end

linkAoAs_full = [];
linkSsAs_full = [];
linkCdAs_full = [];
linkClAs_full = [];
linkCsAs_full = [];
linkCnAs_full = [];
linkCfAs_full = [];

yawAngles_full   = [];
pitchAngles_full = [];
ironcubCdAs_full = [];
ironcubClAs_full = [];
ironcubCsAs_full = [];

for jointConfigIndex = 1 : length(fieldnames(data))

    jointConfigName = jointConfigNames{jointConfigIndex};
    jointPos        = data.(jointConfigName).jointConfig * pi/180;

    % idyntree model initialization
    KinDynModel = iDynTreeWrappers.loadReducedModel(jointNames, 'root_link', modelPath, fileName, false);
    iDynTreeWrappers.setRobotState(KinDynModel, basePose, jointPos, baseVel, jointVel, gravAcc);

    % robot visualization
    %             iDynTreeWrappers.prepareVisualization(KinDynModel, meshFilePrefix, 'color', [0.96,0.96,0.96], ...
    %                                                 'material', 'dull', 'transparency', 0.5, 'debug', true, 'view', [-45 5]);

    dummyVector = nan(length(data.(jointConfigName).yawAngle(:)), 1);

    linkAoAs = dummyVector;
    linkSsAs = dummyVector;
    linkCdAs = dummyVector;
    linkClAs = dummyVector;
    linkCsAs = dummyVector;
    
    linkAoAs_spec = dummyVector;
    linkSsAs_spec = dummyVector;
    linkCdAs_spec = dummyVector;
    linkClAs_spec = dummyVector;
    linkCsAs_spec = dummyVector;

    yawAngles   = dummyVector;
    pitchAngles = dummyVector;
    ironcubCdAs = dummyVector;
    ironcubClAs = dummyVector;
    ironcubCsAs = dummyVector;

    for simIndex = 1 : length(data.(jointConfigName).yawAngle(:))

        % adjust robot pose
        yawAngle   = data.(jointConfigName).yawAngle(simIndex);
        pitchAngle = data.(jointConfigName).pitchAngle(simIndex);
        R_yaw      = rotz(yawAngle);
        R_pitch    = roty(pitchAngle - 90);
        w_H_base   = [R_yaw * R_pitch, zeros(3,1);
                           zeros(1,3),         1];
        % Compute link alpha (angle of attack)
        base_H_link       = iDynTreeWrappers.getRelativeTransform(KinDynModel,'root_link',aeroFrameName);
        w_H_link          = w_H_base * base_H_link;
        linkAxisVector    = w_H_link(1:3,1:3) * frameAxis;
        linkAxisVersor    = linkAxisVector/(norm(linkAxisVector) + 1e-6);
        linkAngleOfAttack = acosd(transpose(linkAxisVersor) * [-1; 0; 0]); % [deg]
        % Compute link beta (sideslip angle)
        linkNormalVector  = w_H_link(1:3,1:3) * normalAxis;
        linkNormalVersor  = linkNormalVector/(norm(linkNormalVector) + 1e-6);
        auxVector         = cross([0; -1; 0],linkAxisVersor);
        auxVersor         = auxVector/(norm(auxVector) + 1e-6);
        linkSideslipAngle = acosd(transpose(linkNormalVersor) * auxVersor); % [deg]
        % Store data
        linkAoAs(simIndex) = linkAngleOfAttack;
        linkSsAs(simIndex) = linkSideslipAngle;

        linkCdAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cd'])(simIndex);
        linkClAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cl'])(simIndex);
        linkCsAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cs'])(simIndex);
        
        if linkIndex == 10 || linkIndex == 11 || linkIndex == 12 || linkIndex == 13

            if linkIndex == 10 || linkIndex == 11
                aeroframeName_spec = aeroFrameNames{linkIndex + 2};
                cfdLinkName_spec   = cfdLinkNames{linkIndex + 2};
            elseif linkIndex == 12 || linkIndex == 13
                aeroframeName_spec = aeroFrameNames{linkIndex - 2};
                cfdLinkName_spec   = cfdLinkNames{linkIndex - 2};
            end

            % Compute spec link alpha (angle of attack)
            base_H_link_spec        = iDynTreeWrappers.getRelativeTransform(KinDynModel,'root_link',aeroframeName_spec);
            w_H_link_spec           = w_H_base * base_H_link_spec;
            linkAxisVector_spec     = w_H_link_spec(1:3,1:3) * frameAxis;
            linkAxisVersor_spec     = linkAxisVector_spec/(norm(linkAxisVector_spec) + 1e-6);
            linkAngleOfAttack_spec  = acosd(transpose(linkAxisVersor_spec) * [-1; 0; 0]); % [deg]
            linkAoAs_spec(simIndex) = linkAngleOfAttack_spec;
            % Compute link beta (sideslip angle)
            linkNormalVector_spec  = w_H_link(1:3,1:3) * normalAxis;
            linkNormalVersor_spec  = linkNormalVector_spec/(norm(linkNormalVector_spec) + 1e-6);
            auxVector_spec         = cross([0; -1; 0],linkAxisVersor_spec);
            auxVersor_spec         = auxVector_spec/(norm(auxVector_spec) + 1e-6);
            linkSideslipAngle_spec = acosd(transpose(linkNormalVersor_spec) * auxVersor_spec); % [deg]
            % Store spec link data
            linkCdAs_spec(simIndex) = data.(jointConfigName).([cfdLinkName_spec,'_cd'])(simIndex);
            linkClAs_spec(simIndex) = data.(jointConfigName).([cfdLinkName_spec,'_cl'])(simIndex);
            linkCsAs_spec(simIndex) = data.(jointConfigName).([cfdLinkName_spec,'_cs'])(simIndex);

        end

        if linkIndex == 1
            yawAngles(simIndex)   = yawAngle;
            pitchAngles(simIndex) = pitchAngle;
            ironcubCdAs(simIndex) = data.(jointConfigName).ironcub_cd(simIndex);
            ironcubClAs(simIndex) = data.(jointConfigName).ironcub_cl(simIndex);
            ironcubCsAs(simIndex) = data.(jointConfigName).ironcub_cs(simIndex);
        end

    end

    linkCnAs = sqrt(linkClAs.^2 + linkCsAs.^2);
    linkCfAs = sqrt(linkClAs.^2 + linkCsAs.^2 + linkCdAs.^2);

    linkCnAs_spec = sqrt(linkClAs_spec.^2 + linkCsAs_spec.^2);
    linkCfAs_spec = sqrt(linkClAs_spec.^2 + linkCsAs_spec.^2 + linkCdAs_spec.^2);

    % Build link dataset
    linkAoAs_full = [linkAoAs_full; linkAoAs];
    linkSsAs_full = [linkSsAs_full; linkSsAs];
    linkCdAs_full = [linkCdAs_full; linkCdAs];
    linkClAs_full = [linkClAs_full; linkClAs];
    linkCsAs_full = [linkCsAs_full; linkCsAs];
    linkCnAs_full = [linkCnAs_full; linkCnAs];
    linkCfAs_full = [linkCfAs_full; linkCfAs];

    yawAngles_full   = [yawAngles_full; yawAngles];
    pitchAngles_full = [pitchAngles_full; pitchAngles];
    ironcubCdAs_full = [ironcubCdAs_full; ironcubCdAs];
    ironcubClAs_full = [ironcubClAs_full; ironcubClAs];
    ironcubCsAs_full = [ironcubCsAs_full; ironcubCsAs];

    if linkIndex == 10 || linkIndex == 11 || linkIndex == 12 || linkIndex == 13
        linkAoAs_full = [linkAoAs_full; linkAoAs_spec];
        linkSsAs_full = [linkSsAs_full; linkSsAs_spec];
        linkCdAs_full = [linkCdAs_full; linkCdAs_spec];
        linkClAs_full = [linkClAs_full; linkClAs_spec];
        linkCsAs_full = [linkCsAs_full; linkCsAs_spec];
        linkCnAs_full = [linkCnAs_full; linkCnAs_spec];
        linkCfAs_full = [linkCfAs_full; linkCfAs_spec];
    end

end

%% Generate single link aerodynamic model
X = @(alpha) [ ones(length(alpha),1)        , ... 
               cosd(alpha)                  , ... 
               ...sind(alpha)                  , ... 
               sind(alpha).^2               , ... 
               ...sind(alpha).*cosd(alpha)     , ... 
               cosd(alpha).^3               , ... 
               ...sind(alpha).^3               , ... 
               ...sind(alpha).^2.*cosd(alpha)  , ... 
               ];
alpha_model = transpose(linspace(0,180,1801));

Y1 = linkCdAs_full;
Cd_coefs = X(linkAoAs_full)\Y1;
Cd_coefs_lasso = lasso(X(linkAoAs_full),Y1);
Cd_model    = X(alpha_model)*Cd_coefs;

% Cd_coefs(1) = Cd_coefs(1)+abs(Cd_model(1));
% Cd_model    = X(alpha_model)*Cd_coefs;

Y2 = linkCnAs_full;
Cn_coefs = X(linkAoAs_full)\Y2;
Cn_model = X(alpha_model)*Cn_coefs;

%% Plots
% plot link CdAs vs AoA
fig = figure(linkIndex);
scatter(linkAoAs_full,linkCdAs_full,[],linkSsAs_full); hold on;
plot(alpha_model,Cd_model,'k-','LineWidth',2);
xlabel('$\alpha_{link}$','Interpreter','latex');
ylabel('$C_D A$','Interpreter','latex');
title(cfdLinkName,'Interpreter','none');
grid on;
c = colorbar;
c.Limits = [0 180];
c.Label.Interpreter = 'latex';
c.Label.String = '$\beta_{link}$';
c.Label.Position = [3, 95, 0];
c.Label.Rotation = 0;
c.Label.FontSize = 12;

% plot link ClAs vs AoA
% fig = figure(length(cfdLinkNames)+linkIndex+1);
% scatter(linkAoAs_full,linkClAs_full,[],linkSsAs_full); hold on;
% plot(alpha_model,Cl_model,'k-','LineWidth',2);
% xlabel('$\alpha_{link}$','Interpreter','latex');
% ylabel('$C_L A$','Interpreter','latex');
% title(cfdLinkName,'Interpreter','none');
% grid on;
% c = colorbar;
% c.Limits = [0 180];
% c.Label.Interpreter = 'latex';
% c.Label.String = '$\beta_{link}$';
% c.Label.Position = [3, 95, 0];
% c.Label.Rotation = 0;
% c.Label.FontSize = 12;

% plot link CsAs vs AoA
% fig = figure(2*length(cfdLinkNames)+linkIndex+2);
% scatter(linkAoAs_full,linkCsAs_full,[],linkSsAs_full); hold on;
% xlabel('$\alpha_{link}$','Interpreter','latex');
% ylabel('$C_S A$','Interpreter','latex');
% title(cfdLinkName,'Interpreter','none');
% grid on;
% c = colorbar;
% c.Limits = [0 180];
% c.Label.Interpreter = 'latex';
% c.Label.String = '$\beta_{link}$';
% c.Label.Position = [3, 95, 0];
% c.Label.Rotation = 0;
% c.Label.FontSize = 12;

% plot link CnAs vs AoA
% fig = figure(linkIndex);
% scatter(linkAoAs_full,linkCnAs_full,[],linkSsAs_full); hold on;
% plot(alpha_model,Cn_model,'k-','LineWidth',2);
% xlabel('$\alpha_{link}$','Interpreter','latex');
% ylabel('$C_N A$','Interpreter','latex');
% title(cfdLinkName,'Interpreter','none');
% grid on;
% c = colorbar;
% c.Limits = [0 180];
% c.Label.Interpreter = 'latex';
% c.Label.String = '$\beta_{link}$';
% c.Label.Position = [3, 95, 0];
% c.Label.Rotation = 0;
% c.Label.FontSize = 12;

% plot link CfAs vs AoA
%     fig = figure(linkIndex);
%     scatter(linkAoAs_full,linkCfAs_full,[],linkSsAs_full); hold on;
%     xlabel('$\alpha_{link}$','Interpreter','latex');
%     ylabel('$C_F A$','Interpreter','latex');
%     title(cfdLinkName,'Interpreter','none');
%     grid on;
%     c = colorbar;
%     c.Limits = [0 180];
%     c.Label.Interpreter = 'latex';
%     c.Label.String = '$\beta_{link}$';
%     c.Label.Position = [3, 95, 0];
%     c.Label.Rotation = 0;
%     c.Label.FontSize = 12;


