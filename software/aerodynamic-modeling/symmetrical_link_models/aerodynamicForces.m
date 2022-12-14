close all;
clear all; 
clc;

%% ADD SOURCE PATH
addpath(genpath('./'));            % Adding the main folder path

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

cylinderReynoldsNumber = (airDensity*airSpeed*cylinderDiameter)/airDynamicViscosity;
sphereReynoldsNumber   = (airDensity*airSpeed*sphereDiameter)/airDynamicViscosity;

%% Aerodynamic force coefficients 
re_v = [1e5];
alpha_v = linspace(-180,180,1801);
% Cd = 0*alpha_v;
% Cl = 0*alpha_v;
for ii = 1:length(re_v)
    for i = 1:length(alpha_v)
        [Cd(i,ii), ~, Cl(i,ii)] = cylinderAerodynamicForces(alpha_v(i),re_v(ii),cylinderAspectRatio);
    end
end


%% Plots

figure()
for i = 1:length(re_v)
plot(alpha_v,Cd(:,i),'LineWidth',1.5,'DisplayName',['$Re = $',num2str(re_v(i),3),', $\lambda = $',num2str(cylinderAspectRatio,3)]); hold on;
end
grid on;
ylabel('$C_D$','Interpreter','latex')
xlabel('$\alpha$ [$^\circ$]','Interpreter','latex')
legend('Location','best','Interpreter','latex')

figure()
for i = 1:length(re_v)
plot(alpha_v,Cl(:,i),'LineWidth',1.5,'DisplayName',['$Re = $',num2str(re_v(i),3),', $\lambda = $',num2str(cylinderAspectRatio,3)]); hold on;
end
grid on;
ylabel('$C_L$','Interpreter','latex')
xlabel('$\alpha$ [$^\circ$]','Interpreter','latex')
legend('Location','best','Interpreter','latex')

figure()
for i = 1:length(re_v)
plot(alpha_v,sqrt(Cl(:,i).^2+Cd(:,i).^2),'LineWidth',1.5,'DisplayName',['$Re = $',num2str(re_v(i),3),', $\lambda = $',num2str(cylinderAspectRatio,3)]); hold on;
end
grid on;
ylabel('$C_F$','Interpreter','latex')
xlabel('$\alpha$ [$^\circ$]','Interpreter','latex')
legend('Location','best','Interpreter','latex')

%% REMOVE PATH

rmpath(genpath('./'));            % Removing the main folder path