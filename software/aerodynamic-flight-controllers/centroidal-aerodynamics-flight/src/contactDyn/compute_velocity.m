function [base_linear_velocity, omega_b, s_dot, new_contact] = compute_velocity(M, J_feet, base_pose_dot, s_dot, is_in_contact, Config)

% compute_velocity returns the configuration velocity
% the velocity does not change if there is no impact
% the velocity change if there is the impact

% initialize the jacobian relative to the vertices that will be in contact
J = zeros(1, Config.N_DOF + 6);

new_contact = false;

for ii = 1: 4 * 2
    
    j = (ii - 1) * 3 + 1;
    
    % the impact occurs if the point was not in contact and now it is
    if is_in_contact(ii,2) == 1 && is_in_contact(ii, 1) == 0
     
        % stack the jacobians of the vertices that NOW are in contact
        J           = [J; J_feet(j:j + 2, :)];
        new_contact = true;
    end
end

% if a new contact is detected we should prevent that the velocity of the vertices that previusly were in contact is nonzero.
if new_contact
    
    for ii = 1: 4 * 2
       
        j = (ii - 1) * 3 + 1;
        
        if is_in_contact(ii, 1)
         
            % stack the jacobian of the vertices that WERE in contact
            J = [J; J_feet(j:j + 2, :)];
        end 
    end
end

J = J(2:end, :);

% compute the projection in the null space of the scaled Jacobian of the vertices if a new contact is detected
if ~new_contact
    
    base_linear_velocity = base_pose_dot(1:3);
    omega_b = base_pose_dot(4:6);
    return
else
    
    N = (eye(Config.N_DOF + 6) - M \ (J' * ((J * (M \ J')) \ J)));
    % the velocity after the impact is a function of the velocity before the impact
    % under the constraint that the vertex velocity is equal to zeros
    x = N * [base_pose_dot; s_dot];
    base_linear_velocity = x(1:3);
    omega_b = x(4:6);
    s_dot   = x(7:end);
end
end