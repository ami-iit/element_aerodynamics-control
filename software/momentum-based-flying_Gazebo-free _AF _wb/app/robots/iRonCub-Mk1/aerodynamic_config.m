%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Af_config.v_wind=[10;10;10];%m/s
Af_config.rho=1.225; % kg/m^3 ,air density at 101.325kPa and 15 degree
Af_config.NOL=10; %number of links considered to be added aerodynamics forces
% ['head','chest','r_upper_arm','l_upper_arm','r_elbow_1','l_elbow_1','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg']
Af_config.gama=zeros(Af_config.NOL,1);
Af_config.gama(1:end)=0.23076*0.18693; % section area of chest m^2

Af_config.Ka=Af_config.rho*Af_config.gama*0.5; % Gama coeffient
Af_config.C_D=zeros(Af_config.NOL,1);
Af_config.C_L=zeros(Af_config.NOL,1);
Af_config.C_D(1:end)=10*ones(Af_config.NOL,1); % assume as constant value 
Af_config.C_L(1:end)=10*ones(Af_config.NOL,1); % assume as constant value