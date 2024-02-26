close all;
clear all;
clc;

%% robot data init
jointNames = {'torso_pitch','torso_roll','torso_yaw', ...
    'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
    'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
    'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
    'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

frameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
    'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
    'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};

componentPath  = getenv('IRONCUB_COMPONENT_SOURCE_DIR');
modelPath      = [componentPath,'\models\iRonCub-Mk1\iRonCub\robots\iRonCub-Mk1_Gazebo\'];
fileName       = 'model_stl.urdf';
meshFilePrefix = [componentPath,'\models'];

jointPos    = [0,0,0,-10,25,40,15,-10,25,40,15,0,10,7,0,0,0,0,10,7,0,0,0]* pi/180;

%% set robot state

% N_pitch = 37;
% N_yaw = 37;
% pitchAngles = linspace(-180,180,N_pitch);
% yawAngles = linspace(-180,180,N_yaw);
% 
% for i = 1 : N_pitch
%     for j = 1 : N_yaw
        


        % pitchAngle = pitchAngles(i);
        % yawAngle = yawAngles(j);

        pitchAngle = 0;
        yawAngle = 0;

        % set base Pose according to yaw and pitch angles
        R_yaw     = rotz(yawAngle);
        R_pitch   = roty(pitchAngle);
        A_R_b     = R_yaw * R_pitch;
        basePose  = [     A_R_b, [0.8; 0; 0];
            zeros(1,3),          1];

        %% data for using iDynTreeWrappers functions
        Njoints  = length(jointNames);
        jointVel = zeros(Njoints,1);
        baseVel  = zeros(6,1);
        gravAcc  = [0; 0; 9.81];

        %% robot initialization
        KinDynModel = iDynTreeWrappers.loadReducedModel(jointNames, 'root_link', modelPath, fileName, false);
        iDynTreeWrappers.setRobotState(KinDynModel, basePose, jointPos, baseVel, jointVel, gravAcc);

        %% robot visualization
        iDynTreeWrappers.prepareVisualization(KinDynModel, meshFilePrefix, 'color', [0.9,0.9,0.9], 'material', 'metal', ...
                                                             'transparency', 0.7, 'debug', true, 'view', [-45 20]);

        %% Compute rotation angles

        % b_V = transpose(A_R_b) * [10; 0; 0];
        % 
        % alpha = atan2( [0,0,1]*b_V , [1,0,0]*b_V ) * 180/pi;
        % 
        % % beta  = - asin( [0,1,0]*b_V / norm(b_V) ) * 180/pi;
        % 
        % beta = - atan2( [0,1,0]*b_V , sqrt( ([1,0,0]*b_V)^2 + ([0,0,1]*b_V)^2 ) ) *180/pi; 

        
        % pitchAngle_plot (N_yaw*(i-1)+j) = pitchAngle;
        % yawAngle_plot (N_yaw*(i-1)+j) = yawAngle;
        % alpha_plot (N_yaw*(i-1)+j) = alpha;
        % beta_plot (N_yaw*(i-1)+j) = beta;
        % 
        % alpha_err (N_yaw*(i-1)+j) = alpha - pitchAngle;
        % beta_err (N_yaw*(i-1)+j)  = beta - yawAngle;
        % 
        % %% Compute rotation matrix
        % % all vectors are in body coordinates B
        % xB = [1;0;0];
        % yB = [0;1;0];
        % zB = [0;0;1];
        % 
        % xA = b_V/norm(b_V);
        % 
        % if norm(cross(xA,yB)) ~= 0
        %     zA = cross(xA,yB)*sign(dot(xA,xB));
        % else
        %     zA = nan(3,1);
        % end
        % 
        % % b_z_A = cross(b_x_A,[0;1;0]);
        % 
        % zA = zA / norm(zA);
        % 
        % yA = cross(zA,xA);
        % yA = yA / norm(yA);
        % 
        % b_R_A = [xA, yA, zA];
        % rot_err_iter = norm( A_R_b - transpose(b_R_A));
        % 
        % if (isnan(rot_err_iter)) %|| ~isfinite(rot_err_iter) 
        %     rot_err_iter = 1;
        % end
        % 
        % rot_error (N_yaw*(i-1)+j) = rot_err_iter;

%     end
% end

% figure(1)
% scatter3(pitchAngle_plot,yawAngle_plot,alpha_err);
% xlabel('pitch angle')
% ylabel('yaw angle')
% title('alpha error')
% 
% 
% figure(2)
% scatter3(pitchAngle_plot,yawAngle_plot,beta_err); hold on;
% xlabel('pitch angle')
% ylabel('yaw angle')
% title('beta error')

% figure(3)
% scatter3(pitchAngle_plot,yawAngle_plot,rot_error); hold on;
% xlabel('pitch angle')
% ylabel('yaw angle')
% title('rotation error')