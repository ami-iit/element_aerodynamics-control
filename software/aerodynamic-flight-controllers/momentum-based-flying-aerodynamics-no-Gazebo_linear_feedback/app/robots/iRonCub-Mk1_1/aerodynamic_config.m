%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON aerodynamics related CONFIGURATION PARAMETERS                      %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%rho:air density 1.225 kg/m^3 , at 101.325kPa and 15 degree
%gama : shape coefficient
%Ka : Ka = rho*gama/2
aerodynamics_config.Ka=1;%1;
% we consider the constant coefficient Ka which is related to the robot's shape 
%coefficient and air density as 1,

%identified parameters of aerodynamic models
%cd=c0+c1*sin(a)^2*cos(b)^3+c2*cos(b)^3+c3*cos(b)^2  drag coefficient
%cn=c4+c5*sin(2a)*cos(b)^2 normal force coefficient
aerodynamics_config.C_0=0.1327; 
aerodynamics_config.C_1=-0.0858;
aerodynamics_config.C_2=0.0679;
aerodynamics_config.C_3=0.0949;
aerodynamics_config.C_4=0.0042;
aerodynamics_config.C_5=0.0598;
