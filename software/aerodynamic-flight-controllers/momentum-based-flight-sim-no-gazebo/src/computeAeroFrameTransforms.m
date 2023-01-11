% To make this script work, launch the initMomentumBasedFlight.m first

%% Aerodynamic frames computations
% Initialize robot
jointNames = {'neck_yaw','torso_pitch','torso_roll','torso_yaw','l_jet_turbine','r_jet_turbine', ...
              'l_shoulder_pitch', 'l_shoulder_roll','l_shoulder_yaw','l_arm_ft_sensor','l_elbow','l_elbow_turbine', ...
              'r_shoulder_pitch', 'r_shoulder_roll','r_shoulder_yaw','r_arm_ft_sensor','r_elbow','r_elbow_turbine', ...
              'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
              'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
jointVel = zeros(23,1);
baseVel  = zeros(6,1);
gravAcc  = [0; 0; 9.81];
KinDynModel = iDynTreeWrappers.loadReducedModel(jointNames, 'root_link', robot_config.modelPath, robot_config.fileName, false);

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

    linkFrame_X_linkCoM(:,:,frameIndex) = [ eye(3) zeros(3);
                                                       SkewMat   eye(3)];

    linkFrame_T_linkCoM(:,:,frameIndex) = [ eye(3) linkCoMPos;
                                                       zeros(1,3)      1];
end

save(['./app/robots/',robotName,'/aeroFrameTransforms.mat'], 'linkFrame_T_linkCoM', 'linkFrame_X_linkCoM');