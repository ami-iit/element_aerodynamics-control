close all;
clear all; 
clc;

%% INPUT DATA

% cylinder geometry
cylinderDiameter      = 0.50; % [m]
cylinderLength        = 2; % [m]
cylinderReferenceArea = cylinderDiameter*cylinderLength; % [m^2]
cylinderAspectRatio   = cylinderLength/cylinderDiameter; % [-]

% sphere geometry
sphereDiameter = 0.1929;

% Airflow
airSpeed               = 10;      % [m/s]
angleOfAttack          = 30;      % [deg]
airDensity             = 1.225;   % [kg/m^3]
airDynamicViscosity    = 1.8e-5;  % [N s/m^2] at T = 18 C
relativeWindVelocity   = airSpeed * [1; 0; 0];

cylinderReynoldsNumber = (airDensity*airSpeed*cylinderDiameter)/airDynamicViscosity;
sphereReynoldsNumber   = (airDensity*airSpeed*sphereDiameter)/airDynamicViscosity;

%% Aerodynamic force 

k_vector = [1; -1; 0];

cylinderAxisVersor = k_vector/norm(k_vector);

cylinderAngleOfAttack  = acosd((transpose(cylinderAxisVersor) * relativeWindVelocity) / (norm(relativeWindVelocity) + 1e-9)); % [deg]

[Cd, ~, Cn_sin] = cylinderAerodynamicForces(cylinderAngleOfAttack, cylinderReynoldsNumber, cylinderAspectRatio);

auxiliaryVector = cross(cross(relativeWindVelocity,cylinderAxisVersor),relativeWindVelocity);

cylinderNormalForce = - 0.5 * airDensity * cylinderReferenceArea * Cn_sin * ...
                      cross(cross(relativeWindVelocity,cylinderAxisVersor),relativeWindVelocity);

cylinderDragForce = 0.5 * airDensity * cylinderReferenceArea * norm(relativeWindVelocity) * Cd * relativeWindVelocity;

cylinderAerodynamicForce = cylinderNormalForce + cylinderDragForce;


%% Plot

figure()
hold on
quiver(0,0,cylinderAxisVersor(1),cylinderAxisVersor(2),'k--');
quiver(0,0,cylinderDragForce(1)/norm(cylinderAerodynamicForce), ...
           cylinderDragForce(2)/norm(cylinderAerodynamicForce),'r-');
quiver(0,0,cylinderNormalForce(1)/norm(cylinderAerodynamicForce), ...
       cylinderNormalForce(2)/norm(cylinderAerodynamicForce),'b-');
quiver(0,0,cylinderAerodynamicForce(1)/norm(cylinderAerodynamicForce), ...
       cylinderAerodynamicForce(2)/norm(cylinderAerodynamicForce),'y-');
quiver(-1,0,1,0,'g-');

% cylinderPlot = plot([-cylinderDiameter/2 cylinderDiameter/2 cylinderDiameter/2 -cylinderDiameter/2 -cylinderDiameter/2], ...
%                     [-cylinderLength/2 -cylinderLength/2 cylinderLength/2 cylinderLength/2 -cylinderLength/2],'k--');
% rotate(cylinderPlot,[0 0 1],cylinderAngleOfAttack-90);

axis equal;
grid on;


