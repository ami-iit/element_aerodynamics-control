function [CD, CN, CN_bar] = cylinderAerodynamicForces(angleOfAttack, reynoldsNumber, aspectRatio)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Description: This function calculates the 3D aerodynamic Drag and Normal
%              force coefficients (CD and CN) and the corrected Normal 
%              force coefficient (CN_bar) for a cylindrical isolated link
%              as functions of the angle of attack between the link axis
%              and the relative wind velocity, the Reynolds number of the
%              flow based on the airspeed and link diameter, the link
%              aspect ratio defined as length over diameter of the link
%              itself.
%
% Author: Antonello Paolino
%
% September 2022
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% EXPERIMENTAL DATA 
% (To be initialized in the Config when moved in the .mdl of the controller)
% Cd = Cd(Re) [NASA report N.121, Wieselsberger, 1922], ref area = d*l
Re_exp = [0, 0.116, 0.241, 0.728, 3.18, 8.01, 17.7, 31.7, 63.6, 196, ...
          356, 892, 2.27e3, 5.15e3, 9.18e3, 1.50e4, 3.03e4, 6.55e4, ...
          1.07e5, 1.61e5, 1.76e5, 2.20e5, 2.56e5, 2.93e5, 3.29e5, ... 
          3.65e5, 3.91e5, 4.27e5, 4.56e5, 4.90e5, 5.03e5, 5.63e5, ...
          6.28e5, 6.83e5, 7.35e5, 8.10e5, 1e7];
Cd_exp = [54.1, 54.1, 30.5, 13.2, 4.87, 2.94, 2.12, 1.73, 1.48, 1.27, ...
          1.15, 0.964, 0.886, 0.944, 1.06, 1.14, 1.16, 1.17, 1.17, ...
          1.18, 1.17, 1.06, 0.985, 0.910, 0.827, 0.726, 0.649, 0.547, ...
          0.454, 0.337, 0.302, 0.309, 0.318, 0.328, 0.340, 0.355, 0.355];

% Cd = Cd(1/AR) [NASA report N.121, Wieselsberger, 1922], ref area = d*l
AR_exp = [0, 0.0237, 0.0490, 0.0998, 0.200, 0.341, 0.498, 0.998, 10.0];
Cd_AR  = [1.19, 0.986, 0.926, 0.820, 0.737, 0.744, 0.687, 0.614, 0.614];
Cd_AR_factor = Cd_AR/max(Cd_AR);

% Cd0 = Cd0(AR) [Kritzinger, 2004], ref area = (pi*d^2)/4
AR_exp_ax = [0, 0.0957, 0.192, 0.357, 0.432, 0.595, 0.747, 0.882, 1.06, ...
             1.30, 1.55, 1.81, 1.96, 2.09, 2.47, 2.80, 3.10, 3.37, 3.68, ... 
             3.91, 4.22, 4.41, 10.00];
Cd_AR_ax  = [1.17, 1.17, 1.16, 1.13, 1.12, 1.07, 1.02, 0.971, 0.911, ...
             0.856, 0.830, 0.820, 0.820, 0.819, 0.819, 0.818, 0.818, ...
             0.818, 0.817, 0.817, 0.815, 0.814, 0.814];
Cd_AR_ax_factor = Cd_AR_ax/max(Cd_AR_ax);

%% DRAG COEFFICIENT ANALYTICAL MODEL

% Infinite cylinder drag coefficient
Cd_90  = interp1(Re_exp,Cd_exp,reynoldsNumber,'pchip');

% Aspect ratio correction factor
ar_coeff = interp1(AR_exp,Cd_AR_factor,1/aspectRatio,'pchip');
Cd_90_ar = Cd_90*ar_coeff;

% Cd at alpha=0 with AR and Reynolds number correction coefficients
Cd_0     = interp1(AR_exp_ax,Cd_AR_ax_factor,aspectRatio,'pchip');
Cd_0     = Cd_0 * (pi/4) / aspectRatio; % correction for the ref. areas
Re_ref   = 10^5;
Cd_ref   = interp1(Re_exp,Cd_exp,Re_ref);
Re_coeff = Cd_90/Cd_ref;
Cd_0_Re  = Cd_0 * Re_coeff;

% % Angle of attack model [Hoang, Laneville, Legeron], used just [30, 90],
% % because of the symmetry of the problem
% if angleOfAttack >= 30 && angleOfAttack < 50
%     f_alpha = (sind(angleOfAttack))^1.5 + 0.5*(sind(50-angleOfAttack))^1.5 ;
% elseif angleOfAttack >= 50 && angleOfAttack < 130
%     f_alpha = (sind(angleOfAttack))^1.5 ;
% elseif angleOfAttack >= 130 && angleOfAttack <= 150
%     f_alpha = (sind(angleOfAttack))^1.5 + 0.5*(sind(angleOfAttack-130))^1.5 ;
% end

% %  Angle of attack model [Hoerner, 1965]
% Cd = sind(angleOfAttack)^3+0.02;

%  Angle of attack model as syntesis of [Hoerner, 1965] [Hoang, 2015] and
%  [Kritzinger, 2004] evidences
CD = Cd_0_Re + (Cd_90_ar - Cd_0_Re) * abs(sind(angleOfAttack))^3;


%% NORMAL FORCE COEFFICIENT ANALYTICAL MODEL
% Analytical expression suggested by [Hoerner, 1965]
CN = Cd_90_ar * sind(angleOfAttack)^2 * cosd(angleOfAttack);

CN_bar = Cd_90_ar * sind(angleOfAttack) * cosd(angleOfAttack);

end