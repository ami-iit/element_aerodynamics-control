function gradient = computeSelfCollisionGradient(w_H_i, w_H_j, c_i, c_j, b_c_i, b_c_j, J_i, J_j)

    % COMPUTESELFCOLLISIONGRADIENT compute the gradient of self collision
    %                              constraint between two spheres.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jul. 2022
    % Modified on Oct., 2022
    %

    w_R_i     = w_H_i(1:3,1:3);
    w_R_j     = w_H_j(1:3,1:3);

    % compute analytical jacobians
    Ja_i        = J_i(:,7:end);
    Ja_j        = J_j(:,7:end);

    % compute sphere centers Jacobians
    Jc_i      = Ja_i(1:3,:) - wbc.skew(w_R_i*b_c_i)*Ja_i(4:6,:);
    Jc_j      = Ja_j(1:3,:) - wbc.skew(w_R_j*b_c_j)*Ja_j(4:6,:);

    gradient  = 2*(c_i-c_j)'*(Jc_i-Jc_j);
end