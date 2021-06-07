%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aerodynamics_config.v_wind=[-7.5*sqrt(2)/2;-7.5*sqrt(2)/2;0];%m/s  wind velocity
aerodynamics_config.rho=1.225; % kg/m^3 ,air density at 101.325kPa and 15 degree
aerodynamics_config.NOL=14; %number of links considered to be added aerodynamics forces + total aerodynamic force application frame 'com'
% ['head'=1
%,'chest'=2,'root_link'=3,'r_upper_arm'=4,'l_upper_arm'=5,'r_elbow_1'=6,
%'l_elbow_1'=7,'r_upper_leg'=8,'l_upper_leg'=9,'r_lower_leg'=10,
%'l_lower_leg'=11,'r_foot'=12,'l_foot'=13,'com'=14]
aerodynamics_config.gama=zeros(aerodynamics_config.NOL,1);
% section area is defined as Base X Height which are the parameters of a
% cylinder that is ablt to cover the link geometry
aerodynamics_config.gama(1)=0.22589*0.18996; % head  base X height   m^2
%aerodynamics_config.gama(2)=0.23076*0.18693; % chest
aerodynamics_config.gama(2)=0.46822*0.32683; % chest+turbine
aerodynamics_config.gama(3)=0.23050*0.21007; % root_link
aerodynamics_config.gama(4)=0.10440*0.130; % r_upper_arm
aerodynamics_config.gama(5)=0.10440*0.130; %l_upper_arm
aerodynamics_config.gama(6)=0.17566*0.24622; %r_elbow_1  r_elbow_1_aero_frame
aerodynamics_config.gama(7)=0.17566*0.24622; %l_elbow_1  l_elbow_1_aero_frame
aerodynamics_config.gama(8)=0.130*0.190; %r_upper_leg
aerodynamics_config.gama(9)=0.130*0.190; %l_upper_leg
aerodynamics_config.gama(10)=0.130*0.200; %r_lower_leg
aerodynamics_config.gama(11)=0.130*0.200; %l_lower_leg
aerodynamics_config.gama(12)=0.0950*0.0550; %r_foot
aerodynamics_config.gama(13)=0.0950*0.0550; %l_foot


aerodynamics_config.Ka=aerodynamics_config.rho*aerodynamics_config.gama*0.5; % Gama coeffient Ka=rho*gama/2
aerodynamics_config.Ka(14)=1;% for total aerodynamic force, we consider the refrence area and other constant value as 1,
%the analyzed force coefficients equal to f/v^2

% identified coefficients of the force model for total aerodynamic force
% model: cd=c0+c1*sin(a)^2*cos(beta)^2+c2*cos(beta)^2  (drag) ;
% cn=c3*sin(2a) (normal froce)
aerodynamics_config.C_0=zeros(aerodynamics_config.NOL,1);%
aerodynamics_config.C_1=zeros(aerodynamics_config.NOL,1);%
aerodynamics_config.C_2=zeros(aerodynamics_config.NOL,1);%
aerodynamics_config.C_3=zeros(aerodynamics_config.NOL,1);%

%only the last elements are added value which are related to total
%aerodynamic force
aerodynamics_config.C_0(end)=0.1326; 
aerodynamics_config.C_1(end)=0.0818;
aerodynamics_config.C_2(end)=0.0279;
aerodynamics_config.C_3(end)=0.0376;
