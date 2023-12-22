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

%% %%%%%%%%%%%%%%%%%%%%%%%% DATASET PURPOSE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DATASET_PURPOSE      = 'Full'; % 'Train' | 'Test' | 'Full' |
FORCES_IN_BASE_FRAME = true;

if matches(DATASET_PURPOSE,'Train')
    jointConfigNames = jointConfigNames(1:23);
elseif matches(DATASET_PURPOSE,'Test')
    jointConfigNames = jointConfigNames(24:end);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Aerodynamic forces application points definitions
aeroFrameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
                  'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
                  'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};

cfdLinkNames   = {'head', 'torso', 'left_back_turbine', 'right_back_turbine', ...
                  'left_arm','left_arm_turbine','right_arm','right_arm_turbine',...
                  'root_link','left_leg_upper','left_leg_lower','right_leg_upper','right_leg_lower'};

load([srcPath,'aeroFrameTransforms.mat']);


%% data for iDynTreeWrappers
componentPath  = getenv('IRONCUB_SOFTWARE_SOURCE_DIR');
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

jointPosDeg_full   = [];
yawAngles_full     = [];
pitchAngles_full   = [];
windDirection_full = [];
ironcubCdAs_full   = [];
ironcubClAs_full   = [];
ironcubCsAs_full   = [];

% Initialize progress display
disp('progress: [0%] completed');
tic

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



    for jointConfigIndex = 1: length(jointConfigNames)

        jointConfigName = jointConfigNames{jointConfigIndex};
        jointPosDeg     = data.(jointConfigName).jointConfig;
        jointPosRad     = jointPosDeg * pi/180;
        

        % idyntree model initialization (suppressing disp output)
        result1 = evalc("KinDynModel = iDynTreeWrappers.loadReducedModel(jointNames, 'root_link', modelPath, fileName, false)");
        result2 = evalc("iDynTreeWrappers.setRobotState(KinDynModel, basePose, jointPosRad, baseVel, jointVel, gravAcc)");

        % robot visualization
        %             iDynTreeWrappers.prepareVisualization(KinDynModel, meshFilePrefix, 'color', [0.96,0.96,0.96], ...
        %                                                 'material', 'dull', 'transparency', 0.5, 'debug', true, 'view', [-45 5]);

        dummyVector = nan(length(data.(jointConfigName).yawAngle(:)), 1);
        linkAoAs = dummyVector;
        linkSsAs = dummyVector;
        linkCdAs = dummyVector;
        linkClAs = dummyVector;
        linkCsAs = dummyVector;
        
        jointPosDegs = nan(length(data.(jointConfigName).yawAngle(:)), length(jointPosDeg));
        yawAngles    = dummyVector;
        pitchAngles  = dummyVector;
        ironcubCdAs  = dummyVector;
        ironcubClAs  = dummyVector;
        ironcubCsAs  = dummyVector;

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
                jointPosDegs(simIndex,:) = jointPosDeg;
                yawAngles(simIndex)      = yawAngle;
                pitchAngles(simIndex)    = pitchAngle;
                ironcubCdAs(simIndex)    = data.(jointConfigName).ironcub_cd(simIndex);
                ironcubClAs(simIndex)    = data.(jointConfigName).ironcub_cl(simIndex);
                ironcubCsAs(simIndex)    = data.(jointConfigName).ironcub_cs(simIndex);
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
            jointPosDeg_full = [jointPosDeg_full; jointPosDegs];
        end
        
        %% Status display
        linkStatus = (linkIndex-1)/length(cfdLinkNames) * 100;
        jointConfigStatus = linkStatus + jointConfigIndex/(length(jointConfigNames)*length(cfdLinkNames))*100;
        clc;
        disp(['progress: [',num2str(round(jointConfigStatus)),'%] completed']);

        elapsedTime = toc;
        totalTime = elapsedTime/(jointConfigStatus/100);
        remainingTime = totalTime - elapsedTime;
        disp(['remaining time: [',num2str(round(remainingTime)),'s]']);

    end
    
    linkAoAs_matrix(:,linkIndex) = linkAoAs_full;
    linkSsAs_matrix(:,linkIndex) = linkSsAs_full;
    linkCdAs_matrix(:,linkIndex) = linkCdAs_full;
    linkClAs_matrix(:,linkIndex) = linkClAs_full;
    linkCsAs_matrix(:,linkIndex) = linkCsAs_full;
    linkCnAs_matrix(:,linkIndex) = linkCnAs_full;
    linkCfAs_matrix(:,linkIndex) = linkCfAs_full;

end



for i = 1 : length(pitchAngles_full)

    R_yaw   = rotz(yawAngles_full(i));
    R_pitch = roty(pitchAngles_full(i) - 90);
    A_R_b   = R_yaw * R_pitch;
    b_R_A   = transpose(A_R_b);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %         [       |       |       ]
    % b_R_A = [ b_x_A | b_y_A | b_z_A ]
    %         [       |       |       ]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    windDirection_full(i,:) = b_R_A(:,1); 
    
    if FORCES_IN_BASE_FRAME

        for linkIndex = 1 : length(cfdLinkNames)

            linkAeroForceAreas = [linkCdAs_matrix(i,linkIndex); ...
                                  linkCsAs_matrix(i,linkIndex); ...
                                  linkClAs_matrix(i,linkIndex); ];

            b_linkAeroForceAreas = b_R_A*linkAeroForceAreas;

            linkCdAs_matrix(i,linkIndex) = b_linkAeroForceAreas(1);
            linkCsAs_matrix(i,linkIndex) = b_linkAeroForceAreas(2);
            linkClAs_matrix(i,linkIndex) = b_linkAeroForceAreas(3);

        end

    end

end

%% Save data
if FORCES_IN_BASE_FRAME
    datasetFileName = ['dataset',DATASET_PURPOSE,'.mat'];
else
    datasetFileName = ['dataset',DATASET_PURPOSE,'AeroFrame.mat'];
end

save([srcPath,datasetFileName],'cfdLinkNames','*_matrix','*_full');
                                                % 'linkAoAs_matrix', ...
                                                % 'linkSsAs_matrix', ...
                                                % 'linkCdAs_matrix', ...
                                                % 'linkClAs_matrix', ...
                                                % 'linkCsAs_matrix', ...
                                                % 'linkCnAs_matrix', ...
                                                % 'linkCfAs_matrix', ...
                                                % 'jointPosDeg_full', ...
                                                % 'yawAngles_full', ...
                                                % 'pitchAngles_full', ...
                                                % 'windDirection_full'...
                                                % 'ironcubCdAs_full', ...
                                                % 'ironcubClAs_full', ...
                                                % 'ironcubCsAs_full');
