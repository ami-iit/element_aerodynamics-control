function jointPos_matrix = generateJointPosMatrix(Torso_pitch, Torso_roll, Torso_yaw, ...
                                                  LeftSh_pitch, LeftSh_roll, LeftSh_yaw, LeftElbow, ...
                                                  RightSh_pitch, RightSh_roll, RightSh_yaw, RightElbow, ...
                                                  LeftHip_pitch, LeftHip_roll, LeftHip_yaw, LeftKnee, LeftAnklePitch, LeftAnkleRoll, ...
                                                  RightHip_pitch, RightHip_roll, RightHip_yaw, RightKnee, RightAnklePitch, RightAnkleRoll)
                                              
rng('shuffle')

idx_torsoPitchRoll_i    = randperm(size(Torso_pitch, 1));
idx_torsoPitchRoll_j    = randperm(size(Torso_pitch, 2));
idx_LeftShPitchRoll_i   = randperm(size(LeftSh_pitch, 1));
idx_LeftShPitchRoll_j   = randperm(size(LeftSh_pitch, 2));
idx_RightShPitchRoll_i  = randperm(size(RightSh_pitch, 1));
idx_RightShPitchRoll_j  = randperm(size(RightSh_pitch, 2));
idx_LeftHipPitchRoll_i  = randperm(size(LeftHip_pitch, 1));
idx_LeftHipPitchRoll_j  = randperm(size(LeftHip_pitch, 2));
idx_RigthHipPitchRoll_i = randperm(size(RightHip_pitch, 1));
idx_RigthHipPitchRoll_j = randperm(size(RightHip_pitch, 2));

idx_LeftElbow  = randperm(length(LeftElbow));
idx_RightElbow = randperm(length(RightElbow));
idx_LeftKnee   = randperm(length(LeftKnee));
idx_RightKnee  = randperm(length(RightKnee));

% generate the matrix
jointPos_matrix = zeros(23, length(RightKnee));

% add fixed joints
jointPos_matrix(3,:)  = Torso_yaw;
jointPos_matrix(6,:)  = LeftSh_yaw;
jointPos_matrix(10,:) = RightSh_yaw;
jointPos_matrix(14,:) = LeftHip_yaw;
jointPos_matrix(16,:) = LeftAnklePitch;
jointPos_matrix(17,:) = LeftAnkleRoll;
jointPos_matrix(20,:) = RightHip_yaw;
jointPos_matrix(22,:) = RightAnklePitch;
jointPos_matrix(23,:) = RightAnkleRoll;

for k = 1:length(RightKnee)
    
    % add single joints
    jointPos_matrix(7,k)  = LeftElbow(idx_LeftElbow(k));
    jointPos_matrix(11,k) = RightElbow(idx_RightElbow(k));
    jointPos_matrix(15,k) = LeftKnee(idx_LeftKnee(k));
    jointPos_matrix(21,k) = RightKnee(idx_RightKnee(k));
end

k = 1;

for i = 1:size(Torso_pitch, 1)
    
    for j = 1:size(Torso_pitch, 2)
        
        % add coupled joints
        jointPos_matrix(1,k)  = Torso_pitch(idx_torsoPitchRoll_i(i),idx_torsoPitchRoll_j(j));
        jointPos_matrix(2,k)  = Torso_roll(idx_torsoPitchRoll_i(i),idx_torsoPitchRoll_j(j));
        jointPos_matrix(4,k)  = LeftSh_pitch(idx_LeftShPitchRoll_i(i),idx_LeftShPitchRoll_j(j));
        jointPos_matrix(5,k)  = LeftSh_roll(idx_LeftShPitchRoll_i(i),idx_LeftShPitchRoll_j(j));
        jointPos_matrix(8,k)  = RightSh_pitch(idx_RightShPitchRoll_i(i),idx_RightShPitchRoll_j(j));
        jointPos_matrix(9,k)  = RightSh_roll(idx_RightShPitchRoll_i(i),idx_RightShPitchRoll_j(j));
        jointPos_matrix(12,k) = LeftHip_pitch(idx_LeftHipPitchRoll_i(i),idx_LeftHipPitchRoll_j(j));
        jointPos_matrix(13,k) = LeftHip_roll(idx_LeftHipPitchRoll_i(i),idx_LeftHipPitchRoll_j(j));
        jointPos_matrix(18,k) = RightHip_pitch(idx_RigthHipPitchRoll_i(i),idx_RigthHipPitchRoll_j(j));
        jointPos_matrix(19,k) = RightHip_roll(idx_RigthHipPitchRoll_i(i),idx_RigthHipPitchRoll_j(j));        
        
        k = k +1;
    end
end

end
                                              
                                             