%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aerodynamics_config.v_wind=[10;10;10];%m/s  wind velocity
aerodynamics_config.rho=1.225; % kg/m^3 ,air density at 101.325kPa and 15 degree
aerodynamics_config.NOL=13; %number of links considered to be added aerodynamics forces
% ['head','chest','root_link','r_upper_arm','l_upper_arm','r_elbow_1','l_elbow_1','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot']
aerodynamics_config.gama=zeros(aerodynamics_config.NOL,1);
% section area is defined as Base X Height which are the parameters of a
% cylinder that is ablt to cover the link geometry
aerodynamics_config.gama(1)=0.22589*0.18996; % head  base X height   m^2
aerodynamics_config.gama(2)=0.23076*0.18693; % chest
aerodynamics_config.gama(3)=0.23050*0.21007; % root_link
aerodynamics_config.gama(4)=0.10440*0.130; % r_upper_arm
aerodynamics_config.gama(5)=0.10440*0.130; %l_upper_arm
aerodynamics_config.gama(6)=0.17566*0.24622; %r_elbow_1
aerodynamics_config.gama(7)=0.17566*0.24622; %l_elbow_1
aerodynamics_config.gama(8)=0.130*0.190; %r_upper_leg
aerodynamics_config.gama(9)=0.130*0.190; %l_upper_leg
aerodynamics_config.gama(10)=0.130*0.200; %r_lower_leg
aerodynamics_config.gama(11)=0.130*0.200; %l_lower_leg
aerodynamics_config.gama(12)=0.0950*0.0550; %r_foot
aerodynamics_config.gama(13)=0.0950*0.0550; %l_foot


aerodynamics_config.Ka=aerodynamics_config.rho*aerodynamics_config.gama*0.5; % Gama coeffient Ka=rho*gama/2
aerodynamics_config.C_D=zeros(aerodynamics_config.NOL,1);%drag coefficient
aerodynamics_config.C_L=zeros(aerodynamics_config.NOL,1);%lift coefficient
aerodynamics_config.C_D(1:end)=1*ones(aerodynamics_config.NOL,1); % assume as constant value 
aerodynamics_config.C_L(1:end)=1*ones(aerodynamics_config.NOL,1); % assume as constant value