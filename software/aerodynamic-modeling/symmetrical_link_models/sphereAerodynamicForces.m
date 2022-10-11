function [CD, CL] = sphereAerodynamicForces(reynoldsNumber)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: Antonello Paolino
%
% September 2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ADD SOURCE PATH
% srcPath = './src/';

%% LOAD EXPERIMENTAL DATA 
%Cd = Cd(Re) [NASA report N.121, Wieselsberger, 1922], ref area = d*l
% data_Cd_Re = load([srcPath,'drag-coefficient-sphere.csv']);
% Re_exp     = data_Cd_Re(:,1);
% Cd_exp     = data_Cd_Re(:,2);

% To be initialized in the Config when moved in the .mdl of the controller
Re_exp = [2e4, 7.36e4, 1.54e5, 2.07e5, 3.31e5, 3.42e5, 3.53e5, 3.61e5, ...
          3.66e5, 3.74e5, 3.84e5, 4.44e5, 6.94e5, 1.26e6, 3.05e6, 6e6];
Cd_exp = [0.438, 0.498, 0.517, 0.513, 0.433, 0.367, 0.295, 0.231, 0.187, ...
          0.144, 0.101, 0.0628, 0.0845, 0.123, 0.171, 0.188];

%% DRAG ANALYTICAL MODEL

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

%% LIFT STEADY ZERO VALUE
CL = 0;

end