%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Af_config.v_wind=[100;100;100];
Af_config.rho=1.225; % kg/m^3 ,air density at 101.325kPa and 15 degree
Af_config.gama=0.23076*0.18693; % section area of chest m^2
Af_config.Ka=Af_config.rho*Af_config.gama*0.5; % Gama coeffient
Af_config.C_D_link=0.5; % assume as constant value 
Af_config.C_L_link=0.3; % assume as constant value