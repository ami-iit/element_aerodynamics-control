function [y, A, B, C, D] = computeACKFinputs(baseTwist, M, matrixOfJetsAxes, matrixOfJetsArms, pos_vel_acc_jerk_CoM_des, wind_velocity, Config)

    % compute the robot total mass and gravity
    m                      = M(1,1);
    f_grav                 = m*Config.GRAVITY_ACC*[0; 0; 1; zeros(3,1)];

    % compute momentum references and demux jet axes and arms
    [~, LDot_des, ~, ~] = iRonCubLib_v1.computeMomentumReferences(pos_vel_acc_jerk_CoM_des, m);
    [r_J1, r_J2, r_J3, r_J4, ax_J1, ax_J2, ax_J3, ax_J4] = iRonCubLib_v1.demuxJetAxesAndArms(matrixOfJetsAxes, matrixOfJetsArms);
    
    % multiplier of jetsIntensitiesDot in the angular momentum equations
    % (namely, the last 3 rows of Aj)
    Aj_angular = [wbc.skew(r_J1) * ax_J1, ...
                  wbc.skew(r_J2) * ax_J2, ...
                  wbc.skew(r_J3) * ax_J3, ...
                  wbc.skew(r_J4) * ax_J4];
    
    % multiplier of jetsIntensitiesDot in the linear momentum equations
    % (namely, the first 3 rows of Aj)
    Aj_linear = matrixOfJetsAxes;
    
    % compute matrix Aj
    Aj = [Aj_linear; Aj_angular];
    D = Aj;
    
    % Compute aerodynamic matrix
    rel_wind_velocity = wind_velocity - baseTwist(1:3);
    k_a = 0.5 * 1.225 * norm(rel_wind_velocity)^2;
    A_a = eye(6);
    C = k_a * A_a;

    % Assume zero-derivative of aerodynamic coefficients
    A = zeros(6,6);
    B = zeros(6,4);

    % Assign measurement
    y = LDot_des + f_grav;

end