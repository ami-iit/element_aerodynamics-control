%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aerodynamics_config.v_wind=[0;10;0];%m/s  wind velocity
aerodynamics_config.rho=1.225; % kg/m^3 ,air density at 101.325kPa and 15 degree
aerodynamics_config.NOL=13; %number of links considered to be added aerodynamics forces
% ['head'=1
%,'chest'=2,'root_link'=3,'r_upper_arm'=4,'l_upper_arm'=5,'r_elbow_1'=6,
%'l_elbow_1'=7,'r_upper_leg'=8,'l_upper_leg'=9,'r_lower_leg'=10,
%'l_lower_leg'=11,'r_foot'=12,'l_foot'=13]
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