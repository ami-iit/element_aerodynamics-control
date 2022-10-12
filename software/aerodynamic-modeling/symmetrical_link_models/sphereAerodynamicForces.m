function [CD, CN] = sphereAerodynamicForces(reynoldsNumber)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Description: This function calculates the 3D aerodynamic Drag and Normal
%              force coefficients (CD and CN) for a spherical isolated link
%              as functions of the Reynolds number of the flow based on the
%              airspeed and link diameter.
%
% Author: Antonello Paolino
%
% September 2022
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% LOAD EXPERIMENTAL DATA 
% To be initialized in the Config when moved in the .mdl of the controller
Re_exp = [2e4, 7.36e4, 1.54e5, 2.07e5, 3.31e5, 3.42e5, 3.53e5, 3.61e5, ...
          3.66e5, 3.74e5, 3.84e5, 4.44e5, 6.94e5, 1.26e6, 3.05e6, 6e6];
Cd_exp = [0.438, 0.498, 0.517, 0.513, 0.433, 0.367, 0.295, 0.231, 0.187, ...
          0.144, 0.101, 0.0628, 0.0845, 0.123, 0.171, 0.188];

%% DRAG COEFFICIENT ANALYTICAL MODEL

% Sphere drag coefficient
if reynoldsNumber >= 0 && reynoldsNumber < 10
    CD = 4.1275;
elseif reynoldsNumber >= 10 && reynoldsNumber <= 2e4
    CD = 24/reynoldsNumber * (1 + 0.150*reynoldsNumber^0.681) + 0.407/(1 + 8710/reynoldsNumber);
elseif reynoldsNumber > 2e4 && reynoldsNumber < 6e6
    CD  = interp1(Re_exp,Cd_exp,reynoldsNumber,'pchip');
elseif reynoldsNumber >= 6e6
    CD  = 0.188;
end

%% LIFT COEFFICIENT STEADY ZERO VALUE
CN = 0;

end