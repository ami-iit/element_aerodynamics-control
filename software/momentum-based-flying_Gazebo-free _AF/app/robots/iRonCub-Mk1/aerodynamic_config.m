%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Af_config.v_wind=[10;10;10];
Af_config.rho=1.225; % 1.225kg/m^3 ,air density at 101.325kPa and 15 degree
Af_config.gama=0.23076*0.18693; % section area of chest 
Af_config.Ka=Af_config.rho*Af_config.gama*0.5; % Gama coeffient
Af_config.C_D_link=0.5; % assume as constant value 
Af_config.C_L_link=0.3; % assume as constant value