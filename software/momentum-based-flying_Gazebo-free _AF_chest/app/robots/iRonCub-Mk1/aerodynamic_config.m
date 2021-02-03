%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aerodynamics_config.v_wind=[10;10;10];%m/s
aerodynamics_config.rho=1.225; % kg/m^3 ,air density at 101.325kPa and 15 degree
aerodynamics_config.gama=0.23076*0.18693; % section area of chest m^2
aerodynamics_config.Ka=aerodynamics_config.rho*aerodynamics_config.gama*0.5; % Gama coeffient
aerodynamics_config.C_D_link=1; % assume as constant value 
aerodynamics_config.C_L_link=1; % assume as constant value