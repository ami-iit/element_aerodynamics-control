function [KP_momentum_vector, KD_momentum_vector] = gain_scheduling_linear_momentum(posCoM, posCoM_des, Config)

% original gains
KP_momentum_vector  = [Config.gains.momentum.KP_linear, Config.gains.momentum.KP_angular];
KD_momentum_vector  = [Config.gains.momentum.KD_linear, Config.gains.momentum.KD_angular];

% increase gains if the CoM position error (absolute value) is increasing
% over a user-defined threshold
posCoM_err_abs      = abs(posCoM - posCoM_des);
CoM_error_tolerance = 0.04;

if Config.use_gain_scheduling
    
    for k = 1:length(posCoM_err_abs)
        
        if posCoM_err_abs(k) > CoM_error_tolerance
            
            KP_momentum_vector(k) = KP_momentum_vector(k)*Config.gains_scaling_factor;
            KD_momentum_vector(k) = KD_momentum_vector(k)*sqrt(Config.gains_scaling_factor);
        end
    end 
end
