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

linkAoAs_matrix = [];
linkSsAs_matrix = [];
linkCdAs_matrix = [];
linkClAs_matrix = [];
linkCsAs_matrix = [];
linkCnAs_matrix = [];
linkCfAs_matrix = [];

yawAngles_full   = [];
pitchAngles_full = [];
ironcubCdAs_full = [];
ironcubClAs_full = [];
ironcubCsAs_full = [];

for linkIndex = 1 : length(cfdLinkNames)

    linkAoAs_full = [];
    linkSsAs_full = [];
    linkCdAs_full = [];
    linkClAs_full = [];
    linkCsAs_full = [];
    linkCnAs_full = [];
    linkCfAs_full = [];

    cfdLinkName = cfdLinkNames{linkIndex};
    aeroFrameName = aeroFrameNames{linkIndex};

    if matches(aeroFrameName, {'head','chest','root_link'})
        frameAxis  = [0; 1; 0];
        normalAxis = [1; 0; 0];
    else
        frameAxis  = [0; 0; 1];
        normalAxis = [1; 0; 0];
    end



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
            if norm(auxVersor)>=1 || norm(linkNormalVersor)>=1
                disp(['problem at iter ', num2str(simIndex)])
            end
            linkSideslipAngle = acosd(transpose(linkNormalVersor) * auxVersor); % [deg]
            % Store data
            linkAoAs(simIndex) = linkAngleOfAttack;
            linkSsAs(simIndex) = linkSideslipAngle;

%             if linkIndex == 2
%                 linkCdAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cd'])(simIndex) + ...
%                     data.(jointConfigName).([cfdLinkNames{3},'_cd'])(simIndex) + ...
%                     data.(jointConfigName).([cfdLinkNames{4},'_cd'])(simIndex);
%                 linkClAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cl'])(simIndex) + ...
%                     data.(jointConfigName).([cfdLinkNames{3},'_cl'])(simIndex) + ...
%                     data.(jointConfigName).([cfdLinkNames{4},'_cl'])(simIndex);
%                 linkCsAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cs'])(simIndex) + ...
%                     data.(jointConfigName).([cfdLinkNames{3},'_cs'])(simIndex) + ...
%                     data.(jointConfigName).([cfdLinkNames{4},'_cs'])(simIndex);
%             else
                linkCdAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cd'])(simIndex);
                linkClAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cl'])(simIndex);
                linkCsAs(simIndex) = data.(jointConfigName).([cfdLinkName,'_cs'])(simIndex);
%             end


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

        % Build link dataset
        linkAoAs_full = [linkAoAs_full; linkAoAs];
        linkSsAs_full = [linkSsAs_full; linkSsAs];
        linkCdAs_full = [linkCdAs_full; linkCdAs];
        linkClAs_full = [linkClAs_full; linkClAs];
        linkCsAs_full = [linkCsAs_full; linkCsAs];
        linkCnAs_full = [linkCnAs_full; linkCnAs];
        linkCfAs_full = [linkCfAs_full; linkCfAs];
        
        if linkIndex == 1
            yawAngles_full   = [yawAngles_full; yawAngles];
            pitchAngles_full = [pitchAngles_full; pitchAngles];
            ironcubCdAs_full = [ironcubCdAs_full; ironcubCdAs];
            ironcubClAs_full = [ironcubClAs_full; ironcubClAs];
            ironcubCsAs_full = [ironcubCsAs_full; ironcubCsAs];
        end

    end

    linkAoAs_matrix(:,linkIndex) = linkAoAs_full;
    linkSsAs_matrix(:,linkIndex) = linkSsAs_full;
    linkCdAs_matrix(:,linkIndex) = linkCdAs_full;
    linkClAs_matrix(:,linkIndex) = linkClAs_full;
    linkCsAs_matrix(:,linkIndex) = linkCsAs_full;
    linkCnAs_matrix(:,linkIndex) = linkCnAs_full;
    linkCfAs_matrix(:,linkIndex) = linkCfAs_full;

end

%% Evaluate pre-computed models
% model coefficients
Cd_head_coefs     = [0.00334; 0.0188; 0; 0.0142; -0.0172; 0];
Cn_head_coefs     = [0.00425; 0.0116; 0; 0.0108; -0.0101; 0];
Cd_leg_up_coefs   = [0.000637; 0; 0; 0.0144; 0; 0];
Cn_leg_up_coefs   = [0; 0; 0; 0; 0; 0.0133];
Cd_leg_low_coefs  = [0.000540; -0.00573; 0.0269; -0.00763; 0; 0];
Cn_leg_low_coefs  = [0; 0; 0; 0; 0; 0.0100];

% model functions matrix
model_functions_matrix = @(alpha) [ ones(length(alpha),1) , ... 
                                    cosd(alpha)           , ... 
                                    sind(alpha)           , ... 
                                    sind(alpha).^2        , ... 
                                    cosd(alpha).^3        , ... 
                                    sind(alpha).^2.*cosd(alpha)  , ... 
                                    ];

% Coefficients model
model = @(coefs,alpha) model_functions_matrix(alpha) * coefs;

% head
alpha_head = linkAoAs_matrix(:,1);
Cd_head = model(Cd_head_coefs,alpha_head);
Cn_head = model(Cn_head_coefs,alpha_head);

% left_leg_upper
alpha_left_leg_upper = linkAoAs_matrix(:,10);
Cd_left_leg_upper = model(Cd_leg_up_coefs,alpha_left_leg_upper);
Cn_left_leg_upper = model(Cn_leg_up_coefs,alpha_left_leg_upper);

% left_leg_lower
alpha_left_leg_lower = linkAoAs_matrix(:,11);
Cd_left_leg_lower = model(Cd_leg_low_coefs,alpha_left_leg_lower);
Cn_left_leg_lower = model(Cn_leg_low_coefs,alpha_left_leg_lower);

% right_leg_upper
alpha_right_leg_upper = linkAoAs_matrix(:,12);
Cd_right_leg_upper = model(Cd_leg_up_coefs,alpha_right_leg_upper);
Cn_right_leg_upper = model(Cn_leg_up_coefs,alpha_right_leg_upper);

% right_leg_lower
alpha_right_leg_lower = linkAoAs_matrix(:,13);
Cd_right_leg_lower = model(Cd_leg_low_coefs,alpha_right_leg_lower);
Cn_right_leg_lower = model(Cn_leg_low_coefs,alpha_right_leg_lower);

% Calculate partial coefficients
ironcubCd_partial = ironcubCdAs_full - Cd_head - Cd_left_leg_upper - ...
                    Cd_left_leg_lower - Cd_right_leg_upper - Cd_right_leg_lower;

%% Calculate CdA_0 for the non-yet-modeled links

Cd_0_coefs = [];
for linkIndex = 2 : 9

    X_Cd_0 = @(alpha) [ ones(length(alpha),1)        , ...
                        ...cosd(alpha)                  , ...
                        ...sind(alpha)                  , ...
                        ...sind(alpha).^2               , ...
                        ...sind(alpha).*cosd(alpha)     , ...
                        ...cosd(alpha).^3               , ...
                        sind(alpha).^3               , ...
                        ...sind(alpha).^2.*cosd(alpha)  , ...
                        ];
    
    if linkIndex >= 3 && linkIndex <= 8
        if linkIndex == 3
            linkIndex_spec = linkIndex + 1;
        elseif linkIndex == 4
            linkIndex_spec = linkIndex - 1;
        elseif linkIndex == 5 || linkIndex == 6
            linkIndex_spec = linkIndex + 2;
        elseif linkIndex == 7 || linkIndex == 8
            linkIndex_spec = linkIndex - 2;
        end

        X1 = X_Cd_0(linkAoAs_matrix(:,linkIndex));
        Y1 = linkCdAs_matrix(:,linkIndex);
        X2 = X_Cd_0(linkAoAs_matrix(:,linkIndex_spec));
        Y2 = linkCdAs_matrix(:,linkIndex_spec);
        X_full = [X1; X2];
        Y_full = [Y1; Y2];
    else
        X1 = X_Cd_0(linkAoAs_matrix(:,linkIndex));
        Y1 = linkCdAs_matrix(:,linkIndex);
        X_full = X1;
        Y_full = Y1;
    end

    coefs = X_full\Y_full;
    Cd_0_coefs = [Cd_0_coefs;coefs(1)];

end

%% Generate links drag area aerodynamic model
% X = @(alpha_v) [ ones(length(alpha_v(:,2)),1), sind(alpha_v(:,2)).^3, sind(alpha_v(:,2)).^2.*cosd(alpha_v(:,2)), ...
%                  ones(length(alpha_v(:,3)),1), sind(alpha_v(:,3)).^3, sind(alpha_v(:,3)).^2.*cosd(alpha_v(:,3)), ...
%                  ones(length(alpha_v(:,4)),1), sind(alpha_v(:,4)).^3, sind(alpha_v(:,4)).^2.*cosd(alpha_v(:,4)), ...
%                  ones(length(alpha_v(:,5)),1), sind(alpha_v(:,5)).^3, sind(alpha_v(:,5)).^2.*cosd(alpha_v(:,5)), ...
%                  ones(length(alpha_v(:,6)),1), sind(alpha_v(:,6)).^3, sind(alpha_v(:,6)).^2.*cosd(alpha_v(:,6)), ...
%                  ones(length(alpha_v(:,7)),1), sind(alpha_v(:,7)).^3, sind(alpha_v(:,7)).^2.*cosd(alpha_v(:,7)), ...
%                  ones(length(alpha_v(:,8)),1), sind(alpha_v(:,8)).^3, sind(alpha_v(:,8)).^2.*cosd(alpha_v(:,8)), ...
%                  ones(length(alpha_v(:,9)),1), sind(alpha_v(:,9)).^3, sind(alpha_v(:,9)).^2.*cosd(alpha_v(:,9))];

X = @(alpha_2, alpha_3, alpha_4, alpha_5, alpha_6, alpha_7, alpha_8, alpha_9) ...
               [ ones(length(alpha_2),1), sind(alpha_2).^3, ones(length(alpha_2),1), sind(alpha_3).^3, ...
                 ones(length(alpha_2),1), sind(alpha_4).^3, ones(length(alpha_2),1), sind(alpha_5).^3, ...
                 ones(length(alpha_2),1), sind(alpha_6).^3, ones(length(alpha_2),1), sind(alpha_7).^3, ...
                 ones(length(alpha_2),1), sind(alpha_8).^3, ones(length(alpha_2),1), sind(alpha_9).^3];

alpha_model = transpose(linspace(0,180,1801));

Y1 = ironcubCd_partial;
Y2 = linkCdAs_matrix(:,2);
Y3 = linkCdAs_matrix(:,3);
Y4 = linkCdAs_matrix(:,4);
Y5 = linkCdAs_matrix(:,5);
Y6 = linkCdAs_matrix(:,6);
Y7 = linkCdAs_matrix(:,7);
Y8 = linkCdAs_matrix(:,8);
Y9 = linkCdAs_matrix(:,9);

Y_full = [Y1; Y2; Y3; Y4; Y5; Y6; Y7; Y8; Y9];

X1 = X(linkAoAs_matrix(:,2),linkAoAs_matrix(:,3),linkAoAs_matrix(:,4), ...
       linkAoAs_matrix(:,5),linkAoAs_matrix(:,6),linkAoAs_matrix(:,7), ...
       linkAoAs_matrix(:,8),linkAoAs_matrix(:,9));

X2 = [ ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,2)).^3, zeros(length(linkAoAs_matrix(:,2)),14)];

X3 = [ zeros(length(linkAoAs_matrix(:,2)),2), ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,3)).^3, zeros(length(linkAoAs_matrix(:,2)),12) ];

X4 = [ zeros(length(linkAoAs_matrix(:,2)),4), ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,4)).^3, zeros(length(linkAoAs_matrix(:,2)),10) ];

X5 = [ zeros(length(linkAoAs_matrix(:,2)),6), ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,5)).^3, zeros(length(linkAoAs_matrix(:,2)),8) ];

X6 = [ zeros(length(linkAoAs_matrix(:,2)),8), ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,6)).^3, zeros(length(linkAoAs_matrix(:,2)),6) ];

X7 = [ zeros(length(linkAoAs_matrix(:,2)),10), ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,7)).^3, zeros(length(linkAoAs_matrix(:,2)),4) ];

X8 = [ zeros(length(linkAoAs_matrix(:,2)),12), ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,8)).^3, zeros(length(linkAoAs_matrix(:,2)),2) ];

X9 = [ zeros(length(linkAoAs_matrix(:,2)),14), ones(length(linkAoAs_matrix(:,2)),1), sind(linkAoAs_matrix(:,9)).^3 ];

X_full = [X1; X2; X3; X4; X5; X6; X7; X8; X9];




% equality constraints
Aeq = zeros(3,length(X_full(1,:)));
Aeq(1,3) = 1; Aeq(1,5) = -1;    %   I constraint: same c0 for back turbines
Aeq(2,4) = 1; Aeq(2,6) = -1;    %  II constraint: same c1 for back turbines
Aeq(3,7) = 1; Aeq(3,11) = -1;   % III constraint: same c0 for arms
Aeq(4,8) = 1; Aeq(4,12) = -1;   %  IV constraint: same c1 for arms
Aeq(5,9) = 1; Aeq(5,13) = -1;   %   V constraint: same c0 for arm turbines
Aeq(6,10) = 1; Aeq(6,14) = -1;  %  VI constraint: same c1 for arm turbines

beq = zeros(6,1);

% least square linear optimization
Cd_coefs_lsqlin = lsqlin(X_full,Y_full,[],[],Aeq,beq,zeros(length(X_full(1,:)),1));

Cd_0 = Cd_coefs_lsqlin(1:2:15);
Cd_1 = Cd_coefs_lsqlin(2:2:16);

% Cd_model    = X(alpha_model)*Cd_coefs;

%% Theory vs Data Model Plots

% plot link CdAs vs AoA
% linkIndex = 2;
% fig = figure(linkIndex);
% scatter(linkAoAs_matrix(:,2),linkCdAs_full,[],linkSsAs_full); hold on;
% plot(alpha_model,Cd_model,'k-','LineWidth',2);
% xlabel('$\alpha_{link}$','Interpreter','latex');
% ylabel('$C_D A$','Interpreter','latex');
% title(cfdLinkName,'Interpreter','none');
% grid on;
% c = colorbar;
% c.Limits = [0 180];
% c.Label.Interpreter = 'latex';
% c.Label.String = '$\beta_{link}$';
% c.Label.Position = [3, 95, 0];
% c.Label.Rotation = 0;
% c.Label.FontSize = 12;




