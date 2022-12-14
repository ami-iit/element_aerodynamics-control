%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%            COEFFICIENTS FOR THRUST-RPM-INPUT MODELS                     %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TURBINE SERIAL NUMBERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% We have 8 turbines as of 04-May 2022. Four P160 turbines and Four P220
% all from JetCat. Below are the serial numbers of all the turbines we have

% 1644243, 1644244, 1644245, 1644246

% 220566,  220688,  220689,  220690
%
% From the above list, select the turbine serial numbers that are mounted
% on the robot:
Turbines = iRonCubLib.readJetsSerialNumber();

%% ATMOSPHERIC AIR DENSITY RATIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The turbines perform differently under different atmospheric conditions.
% In particular, higher the air density, higher the thrust produced at the
% same RPM. So, the live air density is evaluated against the standard air
% density at 15 Celsius, 101325 Pascals, and 60% rel.Humidity. The ratio
% alpha_air is used to adjust the thrust

Config.P_air     = 101388;    %Pa
Config.T_air     = 35+273.15; %Kelvin
Config.P_ref     = 101325;    %Pa
Config.T_ref     = 15+273.15; %Kelvin
Config.alpha_air = (Config.P_air/Config.T_air)/(Config.P_ref/Config.T_ref);

%% INPUT-TO-ANGULAR SPEED DYNAMIC MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The structure of the angular speed (w) and input signal (u) dynamic model
% is:
%
%   w_dotdot = Kss*(w - a1*u^b1 - c1) + Kd*w_dot + Kwd*w*w_dot + Kwwd*(w^2)*w_dot;
%
%   This model is defined for a given type of turbine. So, all the
%   P160s have the same coefficients, and all P220s have the same
%   coefficients.
%
% The coefficients vector for the angular speed and input is defined in the
% following way:
%
%   jet_u2rpm_dyn_XX = [a1 , b1 , c1 , Kss , Kd , Kwd , Kwwd];
%
% where XX = LA or RA or LB or RB corresponding to Left Arm, Right Arm,
% Left Back or Right Back of the iRonCub respectively
%
% UNITS:
% angular speed (w) --> kiloRPM
% angular speed derivative (w_dot) --> kiloRPM/sec
% angular speed double derivatice (w_dotdot) --> kiloRPM/sec^2
% Input thruttle signal (u) --> 0 percent to 100 percent

% Coefficients for the kRPM-Input dynamic model
P160_u2rpm_dyn = [19.3889, 0.33333, 33, -3.4037, -8.2504,  0.1365, -0.0007];
P100_u2rpm_dyn = [23.7,    0.33333, 44, -3.0096, -5.0556,  0.0774, -0.000425];
P220_u2rpm_dyn = [17.6664, 0.33333, 35, -4.4632, -14.5496, 0.2883, -0.00165];

switch Turbines{1}
    case {"1644243","1644244","1644245","1644246"}
        Config.jet_u2rpm_dyn_LA = P160_u2rpm_dyn; %P160
    case {"1004101","1004102","1004103","1004104"}
        Config.jet_u2rpm_dyn_LA = P100_u2rpm_dyn; %P100
    case {"220566","220688","220689","220690"}
        Config.jet_u2rpm_dyn_LA = P220_u2rpm_dyn; %P220
    otherwise
        error("The turbine serial number selected for Left Arm in the jet config file is incorrect");
end

switch Turbines{2}
    case {"1644243","1644244","1644245","1644246"}
        Config.jet_u2rpm_dyn_RA = P160_u2rpm_dyn; %P160
    case {"1004101","1004102","1004103","1004104"}
        Config.jet_u2rpm_dyn_RA = P100_u2rpm_dyn; %P100
    case {"220566","220688","220689","220690"}
        Config.jet_u2rpm_dyn_RA = P220_u2rpm_dyn; %P220
    otherwise
        error("The turbine serial number selected for Right Arm in the jet config file is incorrect");
end

Config.jet_u2rpm_dyn_LB = P220_u2rpm_dyn; %P220
Config.jet_u2rpm_dyn_RB = P220_u2rpm_dyn; %P220

%% RPM-TO-THRUST MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The structure of Thrust (T) and angular speed (w) model is:
%
%  T = alpha_air*(a2*w^3 + b2*w^2 + c2*w + d2);
%
% This model is defined for each turbine. So each turbine that we use has
% different set of coefficients
%
% The coefficients for the angular speed to Thrust model are defined in the
% following way:
%
% jet_rpm2T_XX = [a2 , b2 , c2, d2];
%
% where XX = LA or RA or LB or RB corresponding to Left Arm, Right Arm,
% Left Back or Right Back respectively
%
% UNITS:
% angular speed (w) --> kiloRPM
% thrust (T) --> newtons (N)

% Coefficients for the RPM-2-Thrust model.
P1644243_rpm2T = [1.666e-04, -0.01974, 1.402, -27.81]; %1644243 - Identified on 10-Mar-2022
P1644244_rpm2T = [1.593e-04, -0.01660, 1.157, -20.29]; %1644244 - Identified on 01-Apr-2021
P1644245_rpm2T = [1.666e-04, -0.01974, 1.402, -27.81]; %1644245 - Not Identifed
P1644246_rpm2T = [1.564e-04, -0.01754, 1.256, -23.22]; %1644246 - Identified on 10-Mar-2022

P1004101_rpm2T = [6.995e-05, -0.01336, 1.123, -27.71]; %P1004101 - Identified on 26-Apr-2021
P1004102_rpm2T = [7.045e-05, -0.01354, 1.15,  -28.34]; %P1004102 - Identified on 26-Apr-2021
P1004103_rpm2T = [7.148e-05, -0.01367, 1.141, -29.53]; %P1004103 - Identified on 26-Apr-2021
P1004104_rpm2T = [6.768e-05, -0.01288, 1.097, -28.49]; %P1004104 - Identified on 26-Apr-2021

P220566_rpm2T  = [2.876e-04, -0.03308, 2.154, -39.07]; %220566 - Identified on 26-Apr-2021
P220688_rpm2T  = [2.410e-04, -0.02348, 1.523, -26.88]; %220688 - Identified on 26-Apr-2021
P220689_rpm2T  = [2.468e-04, -0.02542, 1.691, -30.49]; %220689 - Identified on 10-Mar-2022
P220690_rpm2T  = [2.573e-04, -0.02776, 1.852, -35.05]; %220690 - Identified on 10-Mar-2022

switch Turbines{1}
    case "1644243"
        Config.jet_rpm2T_LA = P1644243_rpm2T;
    case "1644244"
        Config.jet_rpm2T_LA = P1644244_rpm2T;
    case "1644245"
        Config.jet_rpm2T_LA = P1644245_rpm2T;
    case "1644246"
        Config.jet_rpm2T_LA = P1644246_rpm2T;
    case "1004101"
        Config.jet_rpm2T_LA = P1004101_rpm2T;
    case "1004102"
        Config.jet_rpm2T_LA = P1004102_rpm2T;
    case "1004103"
        Config.jet_rpm2T_LA = P1004103_rpm2T;
    case "1004104"
        Config.jet_rpm2T_LA = P1004104_rpm2T;
    case "220566"
        Config.jet_rpm2T_LA = P220566_rpm2T;
    case "220688"
        Config.jet_rpm2T_LA = P220688_rpm2T;
    case "220689"
        Config.jet_rpm2T_LA = P220689_rpm2T;
    case "220690"
        Config.jet_rpm2T_LA = P220690_rpm2T;
    otherwise
        error("The turbine serial number selected for Left Arm in the jet config file is incorrect");
end

switch Turbines{2}
    case "1644243"
        Config.jet_rpm2T_RA = P1644243_rpm2T;
    case "1644244"
        Config.jet_rpm2T_RA = P1644244_rpm2T;
    case "1644245"
        Config.jet_rpm2T_RA = P1644245_rpm2T;
    case "1644246"
        Config.jet_rpm2T_RA = P1644246_rpm2T;
    case "1004101"
        Config.jet_rpm2T_RA = P1004101_rpm2T;
    case "1004102"
        Config.jet_rpm2T_RA = P1004102_rpm2T;
    case "1004103"
        Config.jet_rpm2T_RA = P1004103_rpm2T;
    case "1004104"
        Config.jet_rpm2T_RA = P1004104_rpm2T;
    case "220566"
        Config.jet_rpm2T_RA = P220566_rpm2T;
    case "220688"
        Config.jet_rpm2T_RA = P220688_rpm2T;
    case "220689"
        Config.jet_rpm2T_RA = P220689_rpm2T;
    case "220690"
        Config.jet_rpm2T_RA = P220690_rpm2T;
    otherwise
        error("The turbine serial number selected for Right Arm in the jet config file is incorrect");
end

switch Turbines{3}
    case "220566"
        Config.jet_rpm2T_LB = P220566_rpm2T;
    case "220688"
        Config.jet_rpm2T_LB = P220688_rpm2T;
    case "220689"
        Config.jet_rpm2T_LB = P220689_rpm2T;
    case "220690"
        Config.jet_rpm2T_LB = P220690_rpm2T;
    otherwise
        error("The turbine serial number selected for Left Back in the jet config file is incorrect");
end

switch Turbines{4}
    case "220566"
        Config.jet_rpm2T_RB = P220566_rpm2T;
    case "220688"
        Config.jet_rpm2T_RB = P220688_rpm2T;
    case "220689"
        Config.jet_rpm2T_RB = P220689_rpm2T;
    case "220690"
        Config.jet_rpm2T_RB = P220690_rpm2T;
    otherwise
        error("The turbine serial number selected for Right Back in the jet config file is incorrect");
end

%% THRUST-TO-RPM MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The structure of thrust (T) to RPM (w) model is:
%  w = a*(T/alpha_air)^b + c*(T/alpha_air)
%
% This model is defined for each turbine. So each turbine has a different
% set of coefficients.
%
% The coefficients are defined in the following way:
%
% jet_T2rpm_XX = [a, b, c];
%
% where XX = LA or RA or LB or RB corresponding to Left Arm, Right Arm,
% Left Back or Right Back respectively
%
% UNITS:
% angular speed (w) --> kiloRPM
% thrust (T) --> newtons (N)
%
% Coefficients for the RPM-2-Thrust model. COMMENT OUT the turbines NOT
% mounted on the iRonCub

P1644243_T2rpm = [18.66, 0.4053, -0.1373]; %1644243 - Identified on 10-Mar-2022
P1644244_T2rpm = [15.48, 0.4568, -0.226];  %1644244 - Identified on 01-Apr-2021
P1644245_T2rpm = [18.66, 0.4053, -0.1373]; %1644245 - Not Identifed
P1644246_T2rpm = [15.76, 0.4593, -0.2417]; %1644246 - Identified on 10-Mar-2022

P1004101_T2rpm = [30.29, 0.4301, -0.5943]; %P1004101 - Identified on 26-Apr-2021
P1004102_T2rpm = [28.66, 0.4494, -0.6751]; %P1004102 - Identified on 26-Apr-2021
P1004103_T2rpm = [40.8,  0.306 , -0.0364]; %P1004103 - Identified on 26-Apr-2021
P1004104_T2rpm = [36.49, 0.3498, -0.1965]; %P1004104 - Identified on 26-Apr-2021
   
P220566_T2rpm  = [14.74, 0.424,  -0.1288]; %220566 - Identified on 26-Apr-2021
P220688_T2rpm  = [14.95, 0.4201, -0.1213]; %220688 - Identified on 26-Apr-2021
P220689_T2rpm  = [14.41, 0.4304, -0.1330]; %220689 - Identified on 10-Mar-2022
P220690_T2rpm  = [15.66, 0.407,  -0.1033]; %220690 - Identified on 10-Mar-2022

switch Turbines{1}
    case "1644243"
        Config.jet_T2rpm_LA = P1644243_T2rpm; 
    case "1644244"
        Config.jet_T2rpm_LA = P1644244_T2rpm; 
    case "1644245"
        Config.jet_T2rpm_LA = P1644245_T2rpm;
    case "1644246"
        Config.jet_T2rpm_LA = P1644246_T2rpm; 
    case "1004101"
        Config.jet_T2rpm_LA = P1004101_T2rpm;
    case "1004102"
        Config.jet_T2rpm_LA = P1004102_T2rpm; 
    case "1004103"
        Config.jet_T2rpm_LA = P1004103_T2rpm; 
    case "1004104"
        Config.jet_T2rpm_LA = P1004104_T2rpm; 
    case "220566"
        Config.jet_T2rpm_LA = P220566_T2rpm; 
    case "220688"
        Config.jet_T2rpm_LA = P220688_T2rpm; 
    case "220689"
        Config.jet_T2rpm_LA = P220689_T2rpm; 
    case "220690"
        Config.jet_T2rpm_LA = P220690_T2rpm; 
    otherwise
        error("The turbine serial number selected for Left Arm in the jet config file is incorrect");
end

switch Turbines{2}
    case "1644243"
        Config.jet_T2rpm_RA = P1644243_T2rpm; 
    case "1644244"
        Config.jet_T2rpm_RA = P1644244_T2rpm; 
    case "1644245"
        Config.jet_T2rpm_RA = P1644245_T2rpm;
    case "1644246"
        Config.jet_T2rpm_RA = P1644246_T2rpm; 
    case "1004101"
        Config.jet_T2rpm_RA = P1004101_T2rpm;
    case "1004102"
        Config.jet_T2rpm_RA = P1004102_T2rpm; 
    case "1004103"
        Config.jet_T2rpm_RA = P1004103_T2rpm; 
    case "1004104"
        Config.jet_T2rpm_RA = P1004104_T2rpm; 
    case "220566"
        Config.jet_T2rpm_RA = P220566_T2rpm; 
    case "220688"
        Config.jet_T2rpm_RA = P220688_T2rpm; 
    case "220689"
        Config.jet_T2rpm_RA = P220689_T2rpm; 
    case "220690"
        Config.jet_T2rpm_RA = P220690_T2rpm; 
    otherwise
        error("The turbine serial number selected for Right Arm in the jet config file is incorrect");
end

switch Turbines{3}
    case "220566"
        Config.jet_T2rpm_LB = P220566_T2rpm; 
    case "220688"
        Config.jet_T2rpm_LB = P220688_T2rpm; 
    case "220689"
        Config.jet_T2rpm_LB = P220689_T2rpm; 
    case "220690"
        Config.jet_T2rpm_LB = P220690_T2rpm; 
    otherwise
        error("The turbine serial number selected for Left Back in the jet config file is incorrect");
end

switch Turbines{4}
    case "220566"
        Config.jet_T2rpm_RB = P220566_T2rpm; 
    case "220688"
        Config.jet_T2rpm_RB = P220688_T2rpm; 
    case "220689"
        Config.jet_T2rpm_RB = P220689_T2rpm; 
    case "220690"
        Config.jet_T2rpm_RB = P220690_T2rpm; 
    otherwise
        error("The turbine serial number selected for Right Back in the jet config file is incorrect");
end

%% INPUT-TO-OMEGA_DESIRED STATIC MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This model takes the input signal u that belongs to the interval
% [0,100] and transforms it into the steady-state angular speed of the
% jetcat turbojet engine in kiloRPM
%
% This function is valid only for ThrStickCurve value on the jetcat GSU set
% to 3.0
%
% This model is defined for each turbine type. So all P160 turbines have
% the same set of coefficients; and all the P220 turbines have the same set
% of coefficients that are different from P160
%
% w_ss = p.u^q + w_idle
%
% The coefficients are defined in the following way:
%
% jet_u2rpm_ss_XX = [p, q, w_idle];

% where XX = LA or RA or LB or RB corresponding to Left Arm, Right Arm,
% Left Back or Right Back respectively
%
% UNITS:
% angular speed (w_ss or w) --> kiloRPM
% input (u) --> 0 percent to 100 percent
%
% Following are the coefficients
P160_u2rpm_ss = [19.3889, 0.33333, 33];
P100_u2rpm_ss = [23.7,    0.33333, 44];
P220_u2rpm_ss = [17.6664, 0.33333, 35];

switch Turbines{1}
    case {"1644243","1644244","1644245","1644246"}
        Config.jet_u2rpm_ss_LA = P160_u2rpm_ss; %P160
    case {"1004101","1004102","1004103","1004104"}
        Config.jet_u2rpm_ss_LA = P100_u2rpm_ss; %P100
    case {"220566","220688","220689","220690"}
        Config.jet_u2rpm_ss_LA = P220_u2rpm_ss; %P220
    otherwise
        error("The turbine serial number selected for Left Arm in the jet config file is incorrect");
end

switch Turbines{2}
    case {"1644243","1644244","1644245","1644246"}
        Config.jet_u2rpm_ss_RA = P160_u2rpm_ss; %P160
    case {"1004101","1004102","1004103","1004104"}
        Config.jet_u2rpm_ss_RA = P100_u2rpm_ss; %P100
    case {"220566","220688","220689","220690"}
        Config.jet_u2rpm_ss_RA = P220_u2rpm_ss; %P220
    otherwise
        error("The turbine serial number selected for Right Arm in the jet config file is incorrect");
end

Config.jet_u2rpm_ss_LB = P220_u2rpm_ss; %P220
Config.jet_u2rpm_ss_RB = P220_u2rpm_ss; %P220

%% OMEGA_DES-TO-INPUT MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This model takes the desired angular speed (w_des) and converts it to the
% input signal 'u' that can be applied to achieve the omega_desired
% angular speed
%
% This model is valid only for ThrStickCurve value on the jetcat GSU set
% to 3.0
%
% This model is defined for each turbine type. So all P160 turbines have
% the same set of coefficients; and all the P220 turbines have the same set
% of coefficients that are different from P160
%
% u = ((w_des-w_idle)/p)^q
%
% The coefficients are defined in the following way:
%
% jet_rpm2u_ss_XX = [p, q, w_idle];
%
% UNITS:
% angular speed (w_ss or w) --> kiloRPM
% input (u) --> 0 percent to 100 percent
%
% Following are the coefficients
P160_rpm2u_ss = [19.3889, 3, 33]; 
P100_rpm2u_ss = [23.7,    3, 44];
P220_rpm2u_ss = [17.6664, 3, 35];

switch Turbines{1}
    case {"1644243","1644244","1644245","1644246"}
        Config.jet_rpm2u_ss_LA = P160_rpm2u_ss; %P160
    case {"1004101","1004102","1004103","1004104"}
        Config.jet_rpm2u_ss_LA = P100_rpm2u_ss; %P100
    case {"220566","220688","220689","220690"}
        Config.jet_rpm2u_ss_LA = P220_rpm2u_ss; %P220
    otherwise
        error("The turbine serial number selected for Left Arm in the jet config file is incorrect");
end

switch Turbines{2}
    case {"1644243","1644244","1644245","1644246"}
        Config.jet_rpm2u_ss_RA = P160_rpm2u_ss; %P160
    case {"1004101","1004102","1004103","1004104"}
        Config.jet_rpm2u_ss_RA = P100_rpm2u_ss; %P100
    case {"220566","220688","220689","220690"}
        Config.jet_rpm2u_ss_RA = P220_rpm2u_ss; %P220
    otherwise
        error("The turbine serial number selected for Right Arm in the jet config file is incorrect");
end

Config.jet_rpm2u_ss_LB = P220_rpm2u_ss; %P220
Config.jet_rpm2u_ss_RB = P220_rpm2u_ss; %P220
