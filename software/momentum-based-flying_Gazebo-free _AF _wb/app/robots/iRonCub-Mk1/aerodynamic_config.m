%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aerodynamics_config.v_wind=[1;1;1];%m/s
aerodynamics_config.rho=1.225; % kg/m^3 ,air density at 101.325kPa and 15 degree
aerodynamics_config.NOL=13; %number of links considered to be added aerodynamics forces
% ['head','chest','root_link','r_upper_arm','l_upper_arm','r_elbow_1','l_elbow_1','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot']
aerodynamics_config.gama=zeros(aerodynamics_config.NOL,1);
aerodynamics_config.gama(1:end)=0.23076*0.18693; % section area of chest m^2

aerodynamics_config.Ka=aerodynamics_config.rho*aerodynamics_config.gama*0.5; % Gama coeffient
aerodynamics_config.C_D=zeros(aerodynamics_config.NOL,1);
aerodynamics_config.C_L=zeros(aerodynamics_config.NOL,1);
aerodynamics_config.C_D(1:end)=1*ones(aerodynamics_config.NOL,1); % assume as constant value 
aerodynamics_config.C_L(1:end)=1*ones(aerodynamics_config.NOL,1); % assume as constant value