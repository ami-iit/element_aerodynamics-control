close all; 
clear all; 
% clc;


%% Data
% Airflow
airSpeed               = 10;      % [m/s]
airDensity             = 1.225;   % [kg/m^3]
airDynamicViscosity    = 1.8e-5;  % [N s/m^2] at T = 18 C
linkRelativeWindVelocity   = airSpeed * [1; 0; 0];

k_vector = [1; 1; 0];

linkAxisVersor = k_vector/norm(k_vector);
angleOfAttack  = acosd((transpose(linkAxisVersor) * -linkRelativeWindVelocity) / (norm(linkRelativeWindVelocity) + 1e-9)); % [deg]

CdA = [1, cosd(angleOfAttack), sind(angleOfAttack).^2, sind(angleOfAttack).^3, cosd(angleOfAttack).^3] * [0.01; 0.0188; 0.0310; -0.0136; -0.0172];
CnA = 0.0578 * sind(angleOfAttack)^2 * cosd(angleOfAttack);

% Corrected coefficient accounting for the cross product normalization term
CnA_bar = 0.0578 * sind(angleOfAttack) * cosd(angleOfAttack);

linkNormalForce        = 0.5 * airDensity * CnA_bar * cross(cross(linkRelativeWindVelocity,linkAxisVersor),linkRelativeWindVelocity);
linkDragForce          = 0.5 * airDensity * norm(linkRelativeWindVelocity) * CdA * linkRelativeWindVelocity;
link_aerodynamic_force = linkNormalForce + linkDragForce;

%% plot
figure()
hold on
quiver(0,0,linkAxisVersor(1),linkAxisVersor(2),'k--');
quiver(0,0,linkDragForce(1)/norm(link_aerodynamic_force), ...
           linkDragForce(2)/norm(link_aerodynamic_force),'r-');
quiver(0,0,linkNormalForce(1)/norm(link_aerodynamic_force), ...
       linkNormalForce(2)/norm(link_aerodynamic_force),'b-');
quiver(0,0,link_aerodynamic_force(1)/norm(link_aerodynamic_force), ...
       link_aerodynamic_force(2)/norm(link_aerodynamic_force),'y-');
quiver(-1,0,1,0,'g-');

axis equal;
grid on;