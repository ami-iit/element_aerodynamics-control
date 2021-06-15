%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON *JETS* CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ẋ  = F(x) + G(x)*u
% xdot(1,:) = Tdot;
% xdot(2,:) = K_T*T + K_TT*T^2 + K_D*Tdot + ... $x^2+e^{\pi i}$
%             K_DD*Tdot^2 + K_TD*T*Tdot + ...
%            (B_U + B_T*T + B_D*Tdot)*(u + B_UU*u^2) + c;

% TODO: Hard Code or import???
%                   K_D        K_DD       K_T        K_TT      K_TD      Bᵤ        Bₜ         B_d       Bᵤᵤ        c
Config.jetC_P100 = [-1.496760, -0.045206, -2.433030, 0.020352, 0.064188, 0.589177, 0.016715, -0.021258, 0.013878, -19.926756];
Config.jetC_P220 = [-0.482993, -0.013562, 1.292093, 0.055923, 0.006887, 0.130662, 0.022564, -0.052168, 0.004485, -5.436267];

% Just rewriting the coefficients in a different data structure: it is
% simpler to handle in the fl controller.

Config.jet.coeff = [Config.jetC_P100; ...
                    Config.jetC_P100; ...
                    Config.jetC_P220; ...
                    Config.jetC_P220];

jets_config.coefficients = [Config.jetC_P100; ...
                            Config.jetC_P100; ...
                            Config.jetC_P220; ...
                            Config.jetC_P220];

jets_config.init_thrust = Config.initialConditions.jets_thrust;

% jets intial conditions
Config.initT = 0.0;
Config.initTdot = 0.0;

% EKF parameters
Config.ekf.initP = [10, 1; 1, 10] * 1e-1;
Config.ekf.process_noise = [10, 1; 1, 10] * 1e-0;
Config.ekf.measurement_noise = 10e2 * 1e-3; %% in simulation there's no noise

% Thrust Gaussian noise, if we need it
% Note that I'm assuming that the noise does not affect the system but just the "measurement"
% If the noise is not zero you should tune the EKF parameters above
Config.thrust_noise = 0.0;

Config.fl.KP = [20, 20, 40, 40] * 0.5;
Config.fl.KI = [15; 15; 30; 30] * 0.5;

Config.jet.u_max = 200;%200;
Config.jet.u_min = 25;
