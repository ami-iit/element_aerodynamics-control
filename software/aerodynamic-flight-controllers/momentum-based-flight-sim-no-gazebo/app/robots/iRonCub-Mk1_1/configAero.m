%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%               AERODYNAMICS CONFIGURATION PARAMETERS                     %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


aero_config.airDensity = 1.225;

%% Robot parameters
aero_config.frameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
                          'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
                          'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};
for frameAxisIndex = 1 : length(aero_config.frameNames)
    if matches(aero_config.frameNames{frameAxisIndex}, {'head','chest','root_link'})
        aero_config.frameAxis(:,frameAxisIndex) = [0; 1; 0];
    else
        aero_config.frameAxis(:,frameAxisIndex) = [0; 0; 1];
    end
end

aero_config.linkDiameters = [0.1929, 0.2467, 0.102, 0.102, ...
                            0.0864, 0.0845, 0.0864, 0.0845, ...
                            0.1476, 0.1256, 0.1197, 0.1256, 0.1197];
aero_config.linkLengths   = [0.1929, 0.1734, 0.2955, 0.2955, ...
                            0.135, 0.223, 0.135, 0.223, ...
                            0.2282, 0.1573, 0.2351, 0.1573, 0.2351];


aero_config.linkReferenceAreas   = aero_config.linkLengths .* aero_config.linkDiameters;
aero_config.linkReferenceAreas(1) = (pi/4) * aero_config.linkReferenceAreas(1);

%% Aerodynamic frames computations

% Initialize robot
modelPath  = 'C:\Users\apaolino\code\component_ironcub\models\iRonCub-Mk1_1\iRonCub\robots\iRonCub-Mk1_1_Gazebo\';
fileName   = 'model.urdf';
meshFilePrefix = 'C:\Users\apaolino\code\component_ironcub\models';
jointNames = {'neck_yaw','torso_pitch','torso_roll','torso_yaw','l_jet_turbine','r_jet_turbine', ...
              'l_shoulder_pitch', 'l_shoulder_roll','l_shoulder_yaw','l_arm_ft_sensor','l_elbow','l_elbow_turbine', ...
              'r_shoulder_pitch', 'r_shoulder_roll','r_shoulder_yaw','r_arm_ft_sensor','r_elbow','r_elbow_turbine', ...
              'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
              'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointVel = zeros(23,1);
baseVel  = zeros(6,1);
gravAcc  = [0; 0; 9.81];
KinDynModel = iDynTreeWrappers.loadReducedModel(jointNames, 'root_link', modelPath, fileName, false);

for frameIndex = 1 : length(aero_config.frameNames)
    frameName = aero_config.frameNames{frameIndex};
    frameID   = KinDynModel.kinDynComp.getFrameIndex(frameName);
    model     = KinDynModel.kinDynComp.getRobotModel();
    link      = model.getLink(frameID);

    linkSpatialInertia   = link.getInertia();
    spatialInertiaVector = linkSpatialInertia.asVector().toMatlab();
    linkCoMPos = spatialInertiaVector(2:4)/spatialInertiaVector(1);

    SkewMat = [0 -linkCoMPos(3) linkCoMPos(2);
               linkCoMPos(3) 0 -linkCoMPos(1);
               -linkCoMPos(2) linkCoMPos(1) 0];

    aero_config.linkFrame_X_linkCoM(:,:,frameIndex) = [ eye(3) zeros(3);
                                                       SkewMat   eye(3)];

    aero_config.linkFrame_T_linkCoM(:,:,frameIndex) = [ eye(3) linkCoMPos;
                                                       zeros(1,3)      1];

%     aero_config.linkFrame_X_linkCoM(:,:,frameIndex) = [ eye(3) zeros(3);
%                                                        zeros(3)   eye(3)];
% 
%     aero_config.linkFrame_T_linkCoM(:,:,frameIndex) = eye(4);
end

%% Cylinder model parameters

% Cd = Cd(Re) [NASA report N.121, Wieselsberger, 1922], ref area = d*l
aero_config.cylinderModel.Re_exp = [0, 0.116, 0.241, 0.728, 3.18, 8.01, 17.7, 31.7, 63.6, 196, ...
                                    356, 892, 2.27e3, 5.15e3, 9.18e3, 1.50e4, 3.03e4, 6.55e4, ...
                                    1.07e5, 1.61e5, 1.76e5, 2.20e5, 2.56e5, 2.93e5, 3.29e5, ...
                                    3.65e5, 3.91e5, 4.27e5, 4.56e5, 4.90e5, 5.03e5, 5.63e5, ...
                                    6.28e5, 6.83e5, 7.35e5, 8.10e5, 1e7];
aero_config.cylinderModel.Cd_exp = [54.1, 54.1, 30.5, 13.2, 4.87, 2.94, 2.12, 1.73, 1.48, 1.27, ...
                                    1.15, 0.964, 0.886, 0.944, 1.06, 1.14, 1.16, 1.17, 1.17, ...
                                    1.18, 1.17, 1.06, 0.985, 0.910, 0.827, 0.726, 0.649, 0.547, ...
                                    0.454, 0.337, 0.302, 0.309, 0.318, 0.328, 0.340, 0.355, 0.355];

% Cd = Cd(1/AR) [NASA report N.121, Wieselsberger, 1922], ref area = d*l
aero_config.cylinderModel.AR_exp       = [0, 0.0237, 0.0490, 0.0998, 0.200, 0.341, 0.498, 0.998, 10.0];
aero_config.cylinderModel.Cd_AR        = [1.19, 0.986, 0.926, 0.820, 0.737, 0.744, 0.687, 0.614, 0.614];
aero_config.cylinderModel.Cd_AR_factor = aero_config.cylinderModel.Cd_AR/max(aero_config.cylinderModel.Cd_AR);

% Cd0 = Cd0(AR) [Kritzinger, 2004], ref area = (pi*d^2)/4
aero_config.cylinderModel.AR_exp_ax       = [0, 0.0957, 0.192, 0.357, 0.432, 0.595, 0.747, 0.882, 1.06, ...
                                             1.30, 1.55, 1.81, 1.96, 2.09, 2.47, 2.80, 3.10, 3.37, 3.68, ... 
                                             3.91, 4.22, 4.41, 10.00];
aero_config.cylinderModel.Cd_AR_ax        = [1.17, 1.17, 1.16, 1.13, 1.12, 1.07, 1.02, 0.971, 0.911, ...
                                             0.856, 0.830, 0.820, 0.820, 0.819, 0.819, 0.818, 0.818, ...
                                             0.818, 0.817, 0.817, 0.815, 0.814, 0.814];
aero_config.cylinderModel.Cd_AR_ax_factor = aero_config.cylinderModel.Cd_AR_ax/max(aero_config.cylinderModel.Cd_AR_ax);


%% Sphere model parameters

% Cd = Cd(Re) [Achenbach, 1972], ref area = pi*(d/2)^2
aero_config.sphereModel.Re_exp   = [2e4, 7.36e4, 1.54e5, 2.07e5, 3.31e5, 3.42e5, 3.53e5, 3.61e5, ...
                                    3.66e5, 3.74e5, 3.84e5, 4.44e5, 6.94e5, 1.26e6, 3.05e6, 6e6];
aero_config.sphereModel.Cd_exp   = [0.438, 0.498, 0.517, 0.513, 0.433, 0.367, 0.295, 0.231, 0.187, ...
                                    0.144, 0.101, 0.0628, 0.0845, 0.123, 0.171, 0.188];
